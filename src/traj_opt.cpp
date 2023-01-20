#include "traj_min_jerk.hpp"
#include "traj_min_snap.hpp"
#include "math_lib.h"

#include <chrono>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <random>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <ros/ros.h>

using namespace std;
using namespace ros;
using namespace Eigen;


VectorXd allocateTime(const MatrixXd &wayPs,
                      double vel,
                      double acc)
{
    int N = (int)(wayPs.cols()) - 1;
    VectorXd durations(N);
    if (N > 0)
    {

        Eigen::Vector3d p0, p1;
        double dtxyz, D, acct, accd, dcct, dccd, t1, t2, t3;
        for (int k = 0; k < N; k++)
        {
            p0 = wayPs.col(k);
            p1 = wayPs.col(k + 1);
            D = (p1 - p0).norm();

            acct = vel / acc;
            accd = (acc * acct * acct / 2);
            dcct = vel / acc;
            dccd = acc * dcct * dcct / 2;

            if (D < accd + dccd)
            {
                t1 = sqrt(acc * D) / acc;
                t2 = (acc * t1) / acc;
                dtxyz = t1 + t2;
            }
            else
            {
                t1 = acct;
                t2 = (D - accd - dccd) / vel;
                t3 = dcct;
                dtxyz = t1 + t2 + t3;
            }

            durations(k) = dtxyz;
        }
    }

    return durations;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example1_node");
    ros::NodeHandle nh_;

    ros::Publisher path_pub = nh_.advertise<nav_msgs::Path>("/minsnaptraj", 10);

    //RandomRouteGenerator routeGen(Array3d(-10, -10, -1), Array3d(10, 10, 1));
    min_snap::SnapOpt snapOpt;
    min_snap::Trajectory minSnapTraj;

    MatrixXd route(3,3);
    route(0,0) = 0;
    route(1,0) = 0;
    route(2,0) = 0;
    route(0,1) = 2;
    route(1,1) = 2;
    route(2,1) = 0;
    route(0,2) = 0;
    route(1,2) = 4;
    route(2,2) = 0;
    
    VectorXd ts;
    Matrix3d iS, fS;
    Eigen::Matrix<double, 3, 4> iSS, fSS;
    iS.setZero();
    fS.setZero();
    Vector3d zeroVec(0.0, 0.0, 0.0);
   
    int i = 2;
    //route = routeGen.generate(i);
    iS.col(0) << route.leftCols<1>();
    fS.col(0) << route.rightCols<1>();
    ts = allocateTime(route, 6.0, 6.0);

    iSS << iS, Eigen::MatrixXd::Zero(3, 1);
    fSS << fS, Eigen::MatrixXd::Zero(3, 1);

    
            
    snapOpt.reset(iSS, fSS, route.cols() - 1);
    snapOpt.generate(route.block(0, 1, 3, i - 1), ts);
    snapOpt.getTraj(minSnapTraj);
    
    double total_time = minSnapTraj.getTotalDuration();
    cout << "total time = " << total_time << endl; 

    nav_msgs::Path path;
    path.header.frame_id = "map";
    int k = 40;
    for(int j=0; j<=k; j++)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = minSnapTraj.getPos(j*total_time/k)(0);
        pose.pose.position.y = minSnapTraj.getPos(j*total_time/k)(1);
        pose.pose.position.z = minSnapTraj.getPos(j*total_time/k)(2);
        path.poses.push_back(pose);
    }
    while(ros::ok())
    {
        path_pub.publish(path);
    }

    ros::spin();

    return 0;
}