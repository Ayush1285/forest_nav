
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "convex_decomp.h"

ros::Publisher cleanmap_pub, window_pub, marker_pub, goal_marker_pub, marker_delete;
geometry_msgs::Pose start_pose, goal_pose;


Eigen::MatrixXi cleanMap(const std::vector<int8_t>& data, u_int32_t height, u_int32_t width)
{
    cv::Mat map_data_img;
    Eigen::MatrixXi ref_map(height, width);
     map_data_img = cv::Mat::zeros(width, height, CV_32F);
     for(size_t y = 0; y < height; y++) 
     {
        for(size_t x = 0; x < width; x++) 
        {
            map_data_img.at<float>(x,y) = data[y * width + x] <= 0 ? 0.0 : 1.0;
        }
    }
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4));
    dilate(map_data_img, map_data_img, element, cv::Point(-1, -1), 4);
    for(size_t y = 0; y < height; y++) 
    {
        for(size_t x = 0; x < width; x++) 
        {
            ref_map(y,x) = 100 * map_data_img.at<float>(x,y);
        }
  }
  return ref_map;
}

PolygonStamped genLocalWindowMsg(const SquareBasic& square, const Pose& origin, float resolution, const string& frame)
{
    PolygonStamped message;
    //std::cout << "Occ Grid frame = " << frame << std::endl;
    message.header.frame_id = frame;
    geometry_msgs::Point32 left_bottom;
    left_bottom.x = (square.centre.x - square.length/2)*resolution + origin.position.x;
    left_bottom.y = (square.centre.y - square.length/2)*resolution + origin.position.y;
    geometry_msgs::Point32 right_bottom;
    right_bottom.x = (square.centre.x + square.length/2)*resolution + origin.position.x;
    right_bottom.y = left_bottom.y;
    geometry_msgs::Point32 right_top;
    right_top.x = right_bottom.x;
    right_top.y = (square.centre.y + square.length/2)*resolution + origin.position.y;
    geometry_msgs::Point32 left_top;
    left_top.x = left_bottom.x;
    left_top.y = right_top.y;
    message.polygon.points.push_back(left_bottom);
    message.polygon.points.push_back(right_bottom);
    message.polygon.points.push_back(right_top);
    message.polygon.points.push_back(left_top);
    return message;
};

nav_msgs::OccupancyGrid genOccupGridMsg
(const nav_msgs::MapMetaData& metadata, const std_msgs::Header& header, const Eigen::MatrixXi& map)
{
    nav_msgs::OccupancyGrid message;
    message.header = header;
    message.info = metadata;
    for(int k = 0; k < map.rows(); k++)
    {
        for(int l = 0; l < map.cols(); l++)
        {
            message.data.push_back(map(k,l));
        }
    }
    return message;
}

visualization_msgs::Marker genMarkerMsg
(const GridPoint& point, const Pose& origin, float resolution, const string& frame)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.pose.position.x = point.x*resolution + origin.position.x;
    marker.pose.position.y = point.y*resolution + origin.position.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    return marker;
}

visualization_msgs::MarkerArray genMarkerArrayMsg
(const std::vector<SquareBasic>& squares, const Pose& origin, float resolution, const string& frame)
{
    /*visualization_msgs::Marker marker0;
    marker0.header.frame_id = frame;
    marker0.header.stamp = ros::Time::now();
    marker0.action = visualization_msgs::Marker::DELETEALL;
    marker_delete.publish(marker0);*/

    visualization_msgs::MarkerArray markerarray;

    for(int i = 0; i<squares.size(); i++)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame;
        marker.header.stamp = ros::Time::now();
        marker.id = i;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.pose.position.x = squares[i].centre.x*resolution + origin.position.x;
        marker.pose.position.y = squares[i].centre.y*resolution + origin.position.y;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = squares[i].length*resolution;
        marker.scale.y = squares[i].length*resolution;
        marker.scale.z = 0.01;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 0.1;
        markerarray.markers.push_back(marker);
    }
    return markerarray;
}
void goalCallback(const geometry_msgs::PoseStampedConstPtr msg)
{
    goal_pose = msg->pose;
    //cout << "Goal set" << endl;
}
void startCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr msg)
{
    start_pose = msg->pose.pose;
    //cout << "Start set" << endl;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr msg)
{
    //cout << "received map" << endl;
    Eigen::MatrixXi clean_map(msg->info.height, msg->info.width);
    nav_msgs::OccupancyGrid map;
    map.header = msg->header;
    map.info = msg->info;
    //GridPoint start(357, 279);
    //GridPoint goal(95, 228);
    GridPoint start, goal;
    /*start.x = (start_pose.position.x - msg->info.origin.position.x)/msg->info.resolution;
    start.y = (start_pose.position.y - msg->info.origin.position.y)/msg->info.resolution;
    goal.x = (goal_pose.position.x - msg->info.origin.position.x)/msg->info.resolution;
    goal.y = (goal_pose.position.y - msg->info.origin.position.y)/msg->info.resolution;*/
    start.x = (-10 - msg->info.origin.position.x)/msg->info.resolution;
    start.y = (-10 - msg->info.origin.position.y)/msg->info.resolution;
    goal.x = (11 - msg->info.origin.position.x)/msg->info.resolution;
    goal.y = (11 - msg->info.origin.position.y)/msg->info.resolution;
    cout << "Start = " << start.x << " " << start.y << endl;
    cout << "Goal = " << goal.x << " " << goal.y << endl;
    clean_map = cleanMap(msg->data, msg->info.height, msg->info.width);
    auto start0 = std::chrono::steady_clock::now();
    std::vector<SquareBasic> squares = convexPlanner(start, goal, clean_map);
    auto end0 = std::chrono::steady_clock::now();
    cout << "Elapsed time in creating squares: " << std::chrono::duration_cast<std::chrono::milliseconds>(end0 - start0).count()
        << " milli sec" << endl;
    cout << "No. of squares = " << squares.size() << endl;
    
    visualization_msgs::MarkerArray markerarray_msg = genMarkerArrayMsg(squares,msg->info.origin, msg->info.resolution, msg->header.frame_id);
    window_pub.publish(markerarray_msg);
    //ros::Duration(0.25).sleep();
    cout << "Published Marker Winow" << endl;
    
    /*auto start0 = std::chrono::steady_clock::now();
    PolygonStamped polygon_msg = genLocalWindowMsg
    (genSquare(start, clean_map), msg->info.origin, msg->info.resolution, msg->header.frame_id);
    auto end0 = std::chrono::steady_clock::now();
    cout << "Elapsed time in creating square: " << std::chrono::duration_cast<std::chrono::microseconds>(end0 - start0).count()
        << " micro sec" << std::endl;*/

    nav_msgs::OccupancyGrid clean_map_msg = genOccupGridMsg(msg->info, msg->header, clean_map);
    //visualization_msgs::Marker marker = genMarkerMsg(start, msg->info.origin, msg->info.resolution, msg->header.frame_id);
    //visualization_msgs::Marker goal_marker = genMarkerMsg(goal, msg->info.origin, msg->info.resolution, msg->header.frame_id);
    while(ros::ok())
    {
        cleanmap_pub.publish(clean_map_msg);
        //marker_pub.publish(marker);
        //goal_marker_pub.publish(goal_marker);
    }
}

int main(int argc, char** argv)
{
    //localmap.start(0) = 401;
    //localmap.start(1) = 296;
    //localmap.goal(0) = 203;
    //localmap.goal(1) = 58;
    ros::init(argc, argv, "testmain");
    ros::NodeHandle nh = ros::NodeHandle("~");
    
    //geometry_msgs::PoseWithCovarianceStampedConstPtr start_msg = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", nh);
    //startCallback(start_msg);
    //geometry_msgs::PoseStampedConstPtr recv_msg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/move_base_simple/goal", nh);
    //goalCallback(recv_msg);
    ros::Subscriber map_subscriber = nh.subscribe("/map", 1, &mapCallback);
    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 2, &goalCallback);
    ros::Subscriber start_sub = nh.subscribe("/initialpose", 2, &startCallback);
    //goal_sub = nh_.subscribe("/move_base_simple/goal", 2, &goalCallback);
    cleanmap_pub = nh.advertise<nav_msgs::OccupancyGrid>("/ref_map",10);
    marker_delete = nh.advertise<visualization_msgs::Marker>("/delete_marker",10);
    window_pub = nh.advertise<visualization_msgs::MarkerArray>("/square_window",10);
    marker_pub = nh.advertise<visualization_msgs::Marker>("/start_marker",5);
    goal_marker_pub = nh.advertise<visualization_msgs::Marker>("/goal_marker",5);
    ros::spin();
}