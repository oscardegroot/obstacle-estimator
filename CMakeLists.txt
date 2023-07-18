cmake_minimum_required(VERSION 2.8.3)
project(obstacle_estimator)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        nav_msgs
        message_generation
        ros_tools
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime ros_tools
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIR}
 include/${PROJECT_NAME}
)



add_executable(obstacle_filter_node nodes/obstacle_filter_node.cpp src/obstacle_filter.cpp src/config.cpp)
target_link_libraries(obstacle_filter_node ${catkin_LIBRARIES})

add_executable(obstacle_prediction_node nodes/obstacle_prediction_node.cpp src/obstacle_prediction.cpp src/config.cpp)
target_link_libraries(obstacle_prediction_node ${catkin_LIBRARIES})

## For debugging
#set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}  -g ")
#set (CMAKE_VERBOSE_MAKEFILE ON)
