#include "rosMsgGen.h"

nav_msgs::OccupancyGrid genOccupGridMsg(const LocalMap& map, const std::string& frame){
    nav_msgs::OccupancyGrid message;
    message.header.frame_id = "uav1/local_origin";
    message.info.height = map.height;
    message.info.width = map.width;
    message.info.origin.position.x = map.origin.x*map.resolution + map.global_origin.position.x;
    message.info.origin.position.y = map.origin.y*map.resolution + map.global_origin.position.y;
    message.info.resolution = map.resolution;
    for(int k = 0; k < map.grid.rows(); k++){
        for(int l = 0; l < map.grid.cols(); l++){
            message.data.push_back(map.grid(k,l));
        }
    }
    return message;
}

geometry_msgs::PolygonStamped genLocalWindowMsg(const GlobalMap& globalmap, const LocalMap& localmap, const std::string& frame){
    geometry_msgs::PolygonStamped message;
    std::cout << "Occ Grid frame = " << frame << std::endl;
    message.header.frame_id = "uav1/local_origin";
    geometry_msgs::Point32 left_bottom;
    left_bottom.x = localmap.origin.x*globalmap.resolution + globalmap.origin.position.x;
    left_bottom.y = localmap.origin.y*globalmap.resolution + globalmap.origin.position.y;
    geometry_msgs::Point32 right_bottom;
    right_bottom.x = left_bottom.x + localmap.width*globalmap.resolution;
    right_bottom.y = left_bottom.y;
    geometry_msgs::Point32 right_top;
    right_top.x = right_bottom.x;
    right_top.y = left_bottom.y + localmap.height*globalmap.resolution;
    geometry_msgs::Point32 left_top;
    left_top.x = left_bottom.x;
    left_top.y = right_top.y;
    message.polygon.points.push_back(left_bottom);
    message.polygon.points.push_back(right_bottom);
    message.polygon.points.push_back(right_top);
    message.polygon.points.push_back(left_top);
    return message;
};

mrs_msgs::Path genMrsPathMsg(const LocalMap& localmap, const Eigen::MatrixX2i& path){
    mrs_msgs::Path message;
    message.header.frame_id = "uav1/local_origin";
    message.fly_now = true;
    message.use_heading = false;
    message.stop_at_waypoints = false;
    message.override_constraints = false;
    //std::cout << "generated total path = " << std::endl;
    int n = 1;
    if(path.rows()-1 > 6){
        //n = path.rows() - 1 - 5;
        n = 2;
    }
    else{
    if(path.rows()-1 > 3){
        n = 1;
    }
    else{if(path.rows()-1 > 2){
        n = 1;
    }}
    }
    for(int i = path.rows() - n-1; i >= 0; i--){
        float yaw;
        //std::cout << path_(i,0) << "  " << path_(i,1) << std::endl;
        if(i==1){
            yaw = std::atan2(path(i,0)-path(i+1,0), path(i,1)-path(i+1,1));
        }
        else{
            yaw = std::atan2(path(i-1,0)-path(i,0), path(i-1,1)-path(i,1));
        }
        mrs_msgs::Reference refpos;
        refpos.position.x = (path(i,1) + localmap.origin.x)*localmap.resolution + localmap.global_origin.position.x;
        refpos.position.y = (path(i,0) + localmap.origin.y)*localmap.resolution + localmap.global_origin.position.y;
        refpos.position.z = 1.5;
        refpos.heading = yaw;
        message.points.push_back(refpos);
    }
    return message;
};

nav_msgs::Path genNavPathMsg(const mrs_msgs::Path& mrs_path){
    tf::Quaternion q;
    nav_msgs::Path nav_path;
    nav_path.header.frame_id = mrs_path.header.frame_id;
    for(int i=0; i<mrs_path.points.size(); i++){
        geometry_msgs::PoseStamped waypoint;
        waypoint.pose.position.x = mrs_path.points[i].position.x;
        waypoint.pose.position.y = mrs_path.points[i].position.y;
        waypoint.pose.position.z = mrs_path.points[i].position.z;
        nav_path.poses.push_back(waypoint);
    }
    return nav_path;
}

visualization_msgs::Marker genMarkerMsg(const GridPoint& point, const GridPoint& local_origin, const geometry_msgs::Pose& global_origin, float resolution){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "uav1/local_origin";
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.pose.position.x = (point.x + local_origin.x)*resolution + global_origin.position.x;
    marker.pose.position.y = (point.y + local_origin.y)*resolution + global_origin.position.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    return marker;
}