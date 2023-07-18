//
// Created by hai on 22/4/19.
//

#ifndef OBSTACLE_KF_H
#define OBSTACLE_KF_H

// General include
// #include <math.h>
// #include <time.h>
#include <obstacle_estimator/config.h>

#include <tf/transform_datatypes.h>

// Message types
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <vector>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <memory>

// Define a class, including a constructor, member variables and member functions
class ObstacleKF
{
public:
  ObstacleKF(int id);
  ~ObstacleKF(){};

  void PoseCallback(const geometry_msgs::PoseStamped &msg);

private:
  void UpdatePosition(const geometry_msgs::PoseStamped &msg);
  void UpdateKF();
  void PublishMessage();

private:
  ros::NodeHandlePtr nh_;

  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Publisher odo_pub_;

  //! Obstacle measurement
  Eigen::Vector3d pos_measured_; // measured position information
  double yaw_measured_;          // measured yaw information

  //! Time information for filter
  ros::Time time_stamp_;          // time stamp of current measurement
  ros::Time time_stamp_previous_; // time stamp of last measurement
  ros::Time dt_publish_;

  double dt_; // time difference between two measurements
  int id_;    // obstacle id

  //! Obstacle estimation and prediction
  Eigen::Matrix<double, 6, 1> state_estimated_;     // estimated state (pos & vel)
  Eigen::Matrix<double, 6, 6> state_cov_estimated_; // estimated covariance matrix

  //! Initializations
  void initializeSubscribers();
  void initializePublishers();
};

#endif // OBSTACLE_ESTIMATOR_OBSTACLE_FILTER_H
