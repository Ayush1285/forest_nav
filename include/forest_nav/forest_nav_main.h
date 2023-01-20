#ifndef FOREST_NAV_MAIN_H
#define FOREST_NAV_MAIN_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <mrs_msgs/PathSrv.h>
#include <mrs_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include "genLocalMap.h"
#include <mrs_lib/transformer.h>


Eigen::Vector3d quat2Euler(const geometry_msgs::Quaternion& msg){
    tf::Quaternion q(
        msg.x,
        msg.y,
        msg.z,
        msg.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    Eigen::Vector3d rpy={roll,pitch,yaw};
    return rpy;
}

bool checkTrajCollision(const LocalMap& localmap);
void planPath();
void hoveringCommand();

ros::Subscriber map_subscriber;
ros::Subscriber odom_subscriber;
ros::Subscriber goal_sub;
ros::Publisher clean_map_pub;
ros::Publisher polygon_pub;
ros::Publisher nav_path_pub;
ros::Publisher start_marker_pub;
ros::Publisher goal_marker_pub;
ros::ServiceClient fly_client;
LocalMap localmap;
Quad quad;
GlobalMap globalmap;
bool plan_flag;
bool collision_flag;
bool first_plan;
mrs_msgs::Path path;
GridPoint prev_local_goal;
GridPoint prev_local_origin;
float goal_x = 50, goal_y = 83;
mrs_lib::Transformer transformer;

#endif

