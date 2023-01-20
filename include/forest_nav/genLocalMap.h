#ifndef FOREST_NAV_GENLOCALMAP_H
#define FOREST_NAV_GENLOCALMAP_H

#include <vector>
#include <stdint.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <memory>
#include "math_lib.h"



Eigen::MatrixXi cleanMapData(const std::vector<int8_t>& map_data, const uint32_t map_height, const uint32_t map_width);
Eigen::MatrixXi cleanMapData(const Eigen::MatrixXi& map_data, const uint32_t map_height, const uint32_t map_width);
GridPoint refineLocalGoal(const Eigen::MatrixXi& globalMap, const GridPoint localgoal_, const GridPoint globalgoal_);
LocalMap genLocalMap(const Quad& quad, const GlobalMap& globalmap_);


int localWindowSize=15;
float max_localgoal_dist = localWindowSize*((3/8)+(1/std::sqrt(2)));

#endif
