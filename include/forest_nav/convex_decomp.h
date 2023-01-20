#ifndef FOREST_NAV_CONVEXDECOMP_H
#define FOREST_NAV_CONVEXDECOMP_H

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
//#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <ros/ros.h>
//#include <tf/tf.h>
//#include <string>
#include <iostream>
#include <set>
#include <chrono>
#include <unordered_map>
#include <memory>
#include "math_lib.h"

using geometry_msgs::PolygonStamped; using std::pair; using std::cout; using std::vector; using std::endl;
using geometry_msgs::Pose; using std::string;

namespace std
{
    template <>
    struct hash<GridPoint>
    {
        size_t operator()( const GridPoint& point ) const
        {
            size_t res = 17;
            res = res * 31 + std::hash<int>()(point.x);
            res = res * 31 + std::hash<int>()(point.y);
            return res;
        }
    };

    template<>
    struct hash<SquareBasic>
    {
        size_t operator()( const SquareBasic& square ) const
        {
            size_t res = 17;
            res = res * 31 + std::hash<int>()(square.centre.x);
            res = res * 31 + std::hash<int>()(square.centre.y);
            res = res * 31 + std::hash<int>()(square.length);
            return res;
        }
    };
    
}

std::unique_ptr<pair<int, GridPoint>> spiralSearch(const GridPoint& centre, const Eigen::MatrixXi& map);
inline Square genSquare(const GridPoint& centre, const GridPoint& parent, const GridPoint& goal);
PolygonStamped genLocalWindowMsg (const SquareBasic& square, const Pose& origin, float resolution, const string& frame);
std::vector<SquareBasic> convexPlanner(const GridPoint& start, const GridPoint& goal, const Eigen::MatrixXi& map);
bool updateObstacleCost(const vector<GridPoint>& obs_list, Square& square);
std::vector<Square> traceConvexPath
(std::unordered_map<SquareBasic, GridPoint>& stored_squares, std::unique_ptr<SquareBasic> curr_square_ptr, const GridPoint& start);

const double pi = std::acos(-1);

#endif