#ifndef FOREST_NAV_ROSMSGGEN_H
#define FOREST_NAV_ROSMSGGEN_H

#include <string>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PolygonStamped.h>
#include <mrs_msgs/Path.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include "genLocalMap.h"

nav_msgs::OccupancyGrid genOccupGridMsg(const LocalMap& map, const std::string& frame);
geometry_msgs::PolygonStamped genLocalWindowMsg(const GlobalMap& globalmap, const LocalMap& localmap, const std::string& frame);
mrs_msgs::Path genMrsPathMsg(const LocalMap& localmap, const Eigen::MatrixX2i& path);
nav_msgs::Path genNavPathMsg(const mrs_msgs::Path& mrs_path);
visualization_msgs::Marker genMarkerMsg(const GridPoint& point, const GridPoint& local_origin, const geometry_msgs::Pose& global_origin, float resolution);

#endif 