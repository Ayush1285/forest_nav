#include <iostream>
#include <chrono>
#include <thread>
#include "forest_nav_main.h"
#include "pathplanner.h"
#include "rosMsgGen.h"
#include <optional>


Eigen::MatrixXi gen2DGrid(const std::vector<int8_t>& data, uint32_t height, u_int32_t width){
    Eigen::MatrixXi map_2Dgrid(height,width);
    for(size_t y = 0; y < height; y++) {
        for(size_t x = 0; x < width; x++) {
      map_2Dgrid(y,x) = data[y * width + x] <= 0 ? 0.0 : 1.0;
        }
    }
    return map_2Dgrid;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr msg){
    //std::cout << "frame of occ grid origin = " << msg->info.origin.header.frame_id << std::endl;
    globalmap.resolution = msg->info.resolution;
    globalmap.width = msg->info.width;
    globalmap.height = msg->info.height;
    /*std::optional<geometry_msgs::Pose> transformed_pose = transformer.transformSingle("uav1/hector_origin", msg->info.origin, "uav1/local_origin");
    if(transformed_pose.has_value())
    {
        globalmap.origin = transformed_pose.value();
        std::cout << "transformed map pose to local origin frame" << std::endl;
        std::cout << "transformed values = " << globalmap.origin.position.x - msg->info.origin.position.x << "  " 
        << globalmap.origin.position.y - msg->info.origin.position.y <<" " << globalmap.origin.position.z - msg->info.origin.position.z << std::endl;
    }
    else{*/
        globalmap.origin = msg->info.origin;
    //}
    //std::cout << "frame of occ grid origin after transform = " << globalmap.origin.header.frame_id << std::endl;
    //auto start0 = std::chrono::steady_clock::now();
    //globalmap.grid = cleanMapData(msg->data, msg->info.height, msg->info.width);
    //auto start0 = std::chrono::steady_clock::now();
    globalmap.grid = gen2DGrid(msg->data, globalmap.height, globalmap.width);
    //auto end0 = std::chrono::steady_clock::now();
    //std::cout << "Elapsed time in generating 2D globalgrid: " << std::chrono::duration_cast<std::chrono::microseconds>(end0 - start0).count()
      //  << " micro sec" << std::endl;
    //start0 = std::chrono::steady_clock::now();
    localmap = genLocalMap(quad, globalmap);
    //end0 = std::chrono::steady_clock::now();
    //std::cout << "Elapsed time in generating localmap: " << std::chrono::duration_cast<std::chrono::microseconds>(end0 - start0).count()
      //  << " micro sec" << std::endl;
    //start0 = std::chrono::steady_clock::now();    
    localmap.grid = cleanMapData(localmap.grid, localmap.height, localmap.width);
    //end0 = std::chrono::steady_clock::now();
    //std::cout << "Elapsed time in cleaning localmap: " << std::chrono::duration_cast<std::chrono::microseconds>(end0 - start0).count()
      //  << " micro sec" << std::endl;
    //auto end0 = std::chrono::steady_clock::now();
    //std::cout << "Elapsed time in cleaning map: " << std::chrono::duration_cast<std::chrono::milliseconds>(end0 - start0).count()
        //<< " ms" << std::endl;
//////////////////////////////////////////////////////////////////////////////////////////////
    if(first_plan){
        collision_flag = true;
        first_plan = false;
        plan_flag = true;
    }
    else{
        collision_flag = checkTrajCollision(localmap);
        //std::cout << "collision flag = " << collision_flag << std::endl;
    }
    nav_msgs::OccupancyGrid cleanmap = genOccupGridMsg(localmap, msg->header.frame_id);
    clean_map_pub.publish(cleanmap);

/////////////////////////////////////////////////////////////////////////////////////////////
    //std::cout << "desired goal = "<< globalmap.goal << std::endl;
    //auto start1 = std::chrono::steady_clock::now();
    //localmap = genLocalMap(quad, globalmap);
    //auto end1 = std::chrono::steady_clock::now();
    //std::cout << "Elapsed time in generating local map: " << std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1).count()
      //  << " ms" << std::endl;
    //bool collision_flag = checkTrajCollision(localmap);

    geometry_msgs::PolygonStamped local_window = genLocalWindowMsg(globalmap, localmap, msg->header.frame_id);
    polygon_pub.publish(local_window);

//////////////////////////////////////////////////////////////////////////////////////////////

    //std::cout << "current x = "<< quad.position(0) << std::endl;
    //std::cout << "current y = "<< quad.position(1) << std::endl;
    if(plan_flag && collision_flag){ 
        visualization_msgs::Marker local_start = genMarkerMsg(localmap.start, localmap.origin, localmap.global_origin, localmap.resolution);
        visualization_msgs::Marker local_goal = genMarkerMsg(localmap.goal, localmap.origin, localmap.global_origin, localmap.resolution);
        start_marker_pub.publish(local_start);
        goal_marker_pub.publish(local_goal);
        std::cout << "replanning path to avoid collision " << std::endl;
        std::thread openPlanningThread(planPath);
        openPlanningThread.join();
    }
}

void planPath(){
    prev_local_goal = localmap.goal;
    prev_local_origin = localmap.origin;
    auto start0 = std::chrono::steady_clock::now();
    AStar astar_planner(localmap);
    auto end0 = std::chrono::steady_clock::now();
    std::cout << "Elapsed time in creating planner object: " << std::chrono::duration_cast<std::chrono::microseconds>(end0 - start0).count()
        << " micro sec" << std::endl;
    start0 = std::chrono::steady_clock::now();
    Eigen::MatrixX2i path_ = astar_planner.returnPath();
    end0 = std::chrono::steady_clock::now();
    std::cout << "Elapsed time in generating path: " << std::chrono::duration_cast<std::chrono::microseconds>(end0 - start0).count()
        << " micro sec" << std::endl;
    std::cout << "returned path = " << path_<< std::endl;

    path = genMrsPathMsg(localmap, path_);
    nav_msgs::Path nav_path = genNavPathMsg(path);
    nav_path_pub.publish(nav_path);
    mrs_msgs::PathSrv fly_srv;
    fly_srv.request.path = path;
    bool flag1 = fly_client.call(fly_srv);
    std::cout << "sent flying path to trajectory generator" << std::endl;    
}

bool checkTrajCollision(const LocalMap& localmap){
    for(int i=0; i < path.points.size(); i++){
        int x = std::floor((path.points[i].position.x-localmap.global_origin.position.x)/localmap.resolution);
        int y = std::floor((path.points[i].position.y-localmap.global_origin.position.y)/localmap.resolution);
        x = x - localmap.origin.x;
        y = y - localmap.origin.y;
        if(localmap.grid(y,x)==100){
            return true;
        }
    }
    return false;
}

void hoveringCommand(){
    mrs_msgs::Path path;
    path.fly_now = true;
    path.use_heading = false;
    path.stop_at_waypoints = false;
    path.override_constraints = false;
    path.override_max_velocity_horizontal = 2.0;
    mrs_msgs::Reference refpos;
    refpos.position.x = quad.position(0);
    refpos.position.y = quad.position(1);
    refpos.position.z = quad.position(2);
    path.points.push_back(refpos);
    mrs_msgs::PathSrv fly_srv;
    fly_srv.request.path = path;
    bool flag1 = fly_client.call(fly_srv);
}

void goalCallback(const geometry_msgs::PoseStampedConstPtr msg){
    globalmap.goal(0) = msg->pose.position.x;
    globalmap.goal(1) = msg->pose.position.y;
    //std::cout << " global goal = "<< globalmap.goal << std::endl;
    plan_flag = true;
    collision_flag = true;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr msg){
//void odomCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr msg){
    geometry_msgs::Quaternion quat  = msg->pose.pose.orientation;
    quad.orientation = quat2Euler(quat);
    quad.position = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
    float dist_to_goal = std::sqrt(std::pow((globalmap.goal(0) - quad.position(0)),2) + std::pow((globalmap.goal(1) - quad.position(1)),2));
    //std::cout << "distance to GLOBAL goal = " << dist_to_goal << std::endl;
    if((dist_to_goal < 0.2) && plan_flag){
        plan_flag  = false;
        hoveringCommand();
    }
    else{
        
        float localgoal_x = (prev_local_goal.x+prev_local_origin.x)*localmap.resolution + localmap.global_origin.position.x;
        float localgoal_y = (prev_local_goal.y+prev_local_origin.y)*localmap.resolution + localmap.global_origin.position.y;
        float dist_to_localgoal = std::sqrt(std::pow((localgoal_x - quad.position(0)),2) + std::pow((localgoal_y - quad.position(1)),2));
        //std::cout << "distance to localgoal = " << dist_to_localgoal << std::endl;
        if(plan_flag && (dist_to_localgoal < 4.0)){
            visualization_msgs::Marker local_start = genMarkerMsg(localmap.start, localmap.origin, localmap.global_origin, localmap.resolution);
            visualization_msgs::Marker local_goal = genMarkerMsg(localmap.goal, localmap.origin, localmap.global_origin, localmap.resolution);
            start_marker_pub.publish(local_start);
            goal_marker_pub.publish(local_goal);
            
            std::thread openPlanningThread(planPath);
            openPlanningThread.join();
            
        }
    }

}



int main(int argc, char** argv){
    
    
    //std::cout << " global goal = "<< globalmap.goal << std::endl;
    plan_flag = false;
    collision_flag = true;
    //plan_flag = true;
    first_plan = true;
    ros::init(argc,argv,"forestnav");
    ros::NodeHandle nh_ = ros::NodeHandle("~");
    transformer = mrs_lib::Transformer(nh_, "forestnav");
    /*nh_.getParam("/forestnav/goal_x", goal_x);
    nh_.getParam("/forestnav/goal_y", goal_y);
    nh_.getParam("/forestnav/local_window_size", localWindowSize);
    nh_.getParam("/forestnav/grids_between_waypoints", segmentLength);*/
    globalmap.goal(0) = goal_x;
    globalmap.goal(1) = goal_y;
    /*ros::ServiceClient constraint_switcher = nh_.serviceClient<mrs_msgs::String>("/uav1/constraint_manager/set_constraints");
    mrs_msgs::String des_constraint;
    des_constraint.request.value = "medium";
    bool return_flag = constraint_switcher.call(des_constraint);*/ 
    fly_client = nh_.serviceClient<mrs_msgs::PathSrv>("/uav1/trajectory_generation/path");
    odom_subscriber = nh_.subscribe("/uav1/odometry/odom_local", 1, &odomCallback);
    //geometry_msgs::PoseStampedConstPtr recv_msg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/move_base_simple/goal", nh_);
    //goalCallback(recv_msg);
    //goal_sub = nh_.subscribe("/move_base_simple/goal", 2, &goalCallback);
    //odom_subscriber = nh_.subscribe("/initialpose",2, &odomCallback);
    
    map_subscriber = nh_.subscribe("/uav1/hector_mapping/map", 1, &mapCallback);
    //map_subscriber = nh_.subscribe("/map", 1, &mapCallback);
    goal_sub = nh_.subscribe("/move_base_simple/goal", 2, &goalCallback);
    
    clean_map_pub = nh_.advertise<nav_msgs::OccupancyGrid>("/clean_map",10);
    polygon_pub = nh_.advertise<geometry_msgs::PolygonStamped>("/local_polygon",5);
    nav_path_pub = nh_.advertise<nav_msgs::Path>("/nav_path",5);
    start_marker_pub = nh_.advertise<visualization_msgs::Marker>("/start_marker",5);
    goal_marker_pub = nh_.advertise<visualization_msgs::Marker>("/goal_marker",5);

    ros::spin();
}
