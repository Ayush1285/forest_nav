cmake_minimum_required(VERSION 3.0.2)
project(forest_nav)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  mrs_msgs
  mrs_lib
  std_srvs
  geometry_msgs
  visualization_msgs
  tf
  tf2
  tf2_ros
)

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})
find_package(OpenCV REQUIRED HINTS /usr/local/lib)

#add_subdirectory(msg)
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  #${CMAKE_CURRENT_BINARY_DIR}/msg
)
link_directories(${GAZEBO_LIBRARY_DIRS} ${SDF_LIBRARY_DIRS})


#add_message_files(
#  FILES
#  SendActuator.msg
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
 # std_msgs
#)

catkin_package(
  CATKIN_DEPENDS std_msgs nav_msgs geometry_msgs mrs_msgs std_srvs visualization_msgs mrs_lib
  DEPENDS Eigen
)

include_directories(
  include
  include/forest_nav
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)


add_library(genlocalmap src/genLocalMap.cpp)
target_link_libraries(genlocalmap ${Eigen_LIBRARIES} ${OpenCV_LIBRARIES})

add_library(pathplanner src/pathplanner.cpp)
target_link_libraries(pathplanner genlocalmap ${Eigen_LIBRARIES})

add_library(rosmsggen src/rosMsgGen.cpp)
target_link_libraries(pathplanner genlocalmap rosmsggen ${Eigen_LIBRARIES})

#add_executable(testfly src/testfly.cpp)
#target_link_libraries(testfly ${catkin_LIBRARIES})

add_library(convexdecomp src/convex_decomp.cpp)
target_link_libraries(convexdecomp ${Eigen_LIBRARIES})

add_executable(testmain src/testmain.cpp)
target_link_libraries(testmain convexdecomp ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})

#add_executable(forestnav src/forest_nav_main.cpp)
#target_link_libraries(forestnav genlocalmap pathplanner rosmsggen ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

#target_link_libraries(speed_publisher command_motor_speed_msgs ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})
#target_link_libraries(testmain genlocalmap pathplanner ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
#add_dependencies(speed_publisher command_motor_speed_msgs quadcopter_generate_messages_cpp)
