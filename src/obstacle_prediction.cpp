/**
 * @file obstacle_prediction.cpp
 * @author Oscar de Groot (o.m.degroot@tudelft.nl) -> Originally by Hai Zhu
 * @brief
 * @version 0.1
 * @date 2023-07-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <obstacle_estimator/obstacle_prediction.h>

#include <ros_tools/helpers.h>

// Constructor:  this will get called whenever an instance of this class is created
ObstacleKF::ObstacleKF(int id)
{
  ROS_INFO_STREAM("KF: Initializing KF for obstacle " << id);
  nh_ = ros::NodeHandlePtr(new ros::NodeHandle);
  id_ = id + 1; // Because mocap starts at "1"

  // Initialization the state covariance, this is important for starting the Kalman filter
  double cov_pos = std::pow(1., 2.);
  double cov_vel = std::pow(1., 2.);
  state_cov_estimated_.setZero();
  state_cov_estimated_(0, 0) = cov_pos;
  state_cov_estimated_(1, 1) = cov_pos;
  state_cov_estimated_(2, 2) = cov_pos;
  state_cov_estimated_(3, 3) = cov_vel;
  state_cov_estimated_(4, 4) = cov_vel;
  state_cov_estimated_(5, 5) = cov_vel;

  // Other initialization
  pos_measured_ = Eigen::Vector3d();
  state_estimated_ = Eigen::Matrix<double, 6, 1>::Zero();

  time_stamp_ = ros::Time::now();
  time_stamp_previous_ = ros::Time::now();
  dt_publish_ = ros::Time::now();

  // Initialization subscriber and publisher
  initializePublishers();
  initializeSubscribers();
}

// Set up subscribers
void ObstacleKF::initializeSubscribers()
{
  if (CONFIG.debug_)
    ROS_INFO("KF: Initializing subscribers");
  sub_ = nh_->subscribe("/mocap_obstacle" + std::to_string(id_) + "/pose", 1, &ObstacleKF::PoseCallback, this);
}

// Set up publisher
void ObstacleKF::initializePublishers()
{
  if (CONFIG.debug_)
    ROS_INFO("KF: Initializing publishers");
  pub_ = nh_->advertise<std_msgs::Float64MultiArray>("/obstacle" + std::to_string(id_) + "/path_prediction", 1);
  odo_pub_ = nh_->advertise<nav_msgs::Odometry>("/obstacle" + std::to_string(id_) + "/state_estimation", 1);
}

// Subscriber callback function
void ObstacleKF::PoseCallback(const geometry_msgs::PoseStamped &msg)
{
  if (CONFIG.debug_)
    ROS_INFO("KF: Subscriber Callback");

  if (time_stamp_previous_ + ros::Duration(1e-3) > msg.header.stamp)
    return;

  time_stamp_ = msg.header.stamp;

  // time difference. If using the node_rate to derive, then comment the following lines
  dt_ = (time_stamp_ - time_stamp_previous_).toSec();

  UpdatePosition(msg);
  UpdateKF();
  PublishMessage();

  // set time
  time_stamp_previous_ = time_stamp_;
}

void ObstacleKF::UpdatePosition(const geometry_msgs::PoseStamped &msg)
{

  // get measured position
  pos_measured_(0) = msg.pose.position.x;
  pos_measured_(1) = msg.pose.position.y;
  pos_measured_(2) = msg.pose.position.z * 0.5; // since a hat is wearing

  // compute the current yaw of the obstacle
  yaw_measured_ = RosTools::quaternionToAngle(msg.pose.orientation);

  if (CONFIG.debug_)
  {
    std::cout << "Position (x = " << pos_measured_(0) << ", y = " << pos_measured_(1) << ", z = " << pos_measured_(2) << ")" << std::endl;
    std::cout << "Orientation (yaw = " << yaw_measured_ << ")" << std::endl;
  }
}

void ObstacleKF::UpdateKF()
{
  // perform Kalman Filtering
  // matrix of state transition model
  Eigen::Matrix<double, 6, 6> A;
  A << 1., 0, 0, dt_, 0, 0, 0, 1., 0, 0, dt_, 0, 0, 0, 1, 0, 0, dt_, 0, 0, 0, 1., 0, 0, 0, 0, 0, 0, 1., 0, 0, 0, 0, 0, 0, 1.;
  Eigen::Matrix<double, 6, 3> B;
  double dt_2 = 0.5 * dt_ * dt_;
  B << dt_2, 0, 0, 0, dt_2, 0, 0, 0, dt_2, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  // matrix of observation model
  Eigen::Matrix<double, 3, 6> H;
  H << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;

  // noise matrix
  double R_pos = 10E-4;
  Eigen::Matrix<double, 3, 3> R; // observation noise covariance
  R << R_pos, 0, 0, 0, R_pos, 0, 0, 0, R_pos;
  double Q_acc = 1000; // mean of noisy acceleration
  double Q_pos = Q_acc * dt_2;
  double Q_vel = Q_acc * dt_;
  Eigen::Matrix<double, 6, 6> Q;
  Q << Q_pos, 0, 0, 0, 0, 0, 0, Q_pos, 0, 0, 0, 0, 0, 0, Q_pos, 0, 0, 0, 0, 0, 0, Q_vel, 0, 0, 0, 0, 0, 0, Q_vel, 0, 0,
      0, 0, 0, 0, Q_vel;

  // prediction
  Eigen::Vector3d u(0, 0, 0);
  state_estimated_ = A * state_estimated_ + B * u;
  state_cov_estimated_ = A * state_cov_estimated_ * A.transpose() + Q;

  // update
  Eigen::Vector3d pos_residual; // pos estimation residual
  pos_residual = pos_measured_ - H * state_estimated_;

  Eigen::Matrix<double, 3, 3> S;
  S = H * state_cov_estimated_ * H.transpose() + R;

  Eigen::Matrix<double, 6, 3> K; // the Kalman gain matrix
  K = (state_cov_estimated_ * H.transpose()) * S.inverse();

  state_estimated_ = state_estimated_ + K * pos_residual; // new estimated state

  Eigen::Matrix<double, 6, 6> I;
  I.setIdentity();                                           // I is an identity matrix
  state_cov_estimated_ = (I - K * H) * state_cov_estimated_; // new covariance

  if (CONFIG.debug_)
    ROS_INFO("KF Updated");

  // prepare published state estimation message
  /*nav_msgs::Odometry est_msg_pub;  // published obstacle state estimation
  est_msg_pub.header = msg.header; // save the header information
  est_msg_pub.pose.pose.orientation = msg.pose.orientation;
  est_msg_pub.pose.pose.position.x = state_estimated_(0); // save the estimated position
  est_msg_pub.pose.pose.position.y = state_estimated_(1);
  est_msg_pub.pose.pose.position.z = state_estimated_(2);
  est_msg_pub.twist.twist.linear.x = state_estimated_(3); // save the estimated velocity
  est_msg_pub.twist.twist.linear.y = state_estimated_(4);
  est_msg_pub.twist.twist.linear.z = state_estimated_(5);

  // save the estimated position and velocity covariance
  // position covariance
  est_msg_pub.pose.covariance[0] = state_cov_estimated_(0, 0);
  est_msg_pub.pose.covariance[1] = state_cov_estimated_(0, 1);
  est_msg_pub.pose.covariance[2] = state_cov_estimated_(0, 2);
  est_msg_pub.pose.covariance[6] = state_cov_estimated_(1, 0);
  est_msg_pub.pose.covariance[7] = state_cov_estimated_(1, 1);
  est_msg_pub.pose.covariance[8] = state_cov_estimated_(1, 2);
  est_msg_pub.pose.covariance[12] = state_cov_estimated_(2, 0);
  est_msg_pub.pose.covariance[13] = state_cov_estimated_(2, 1);
  est_msg_pub.pose.covariance[14] = state_cov_estimated_(2, 2);
  // velocity covariance
  est_msg_pub.twist.covariance[0] = state_cov_estimated_(3, 3);
  est_msg_pub.twist.covariance[1] = state_cov_estimated_(3, 4);
  est_msg_pub.twist.covariance[2] = state_cov_estimated_(3, 5);
  est_msg_pub.twist.covariance[6] = state_cov_estimated_(4, 3);
  est_msg_pub.twist.covariance[7] = state_cov_estimated_(4, 4);
  est_msg_pub.twist.covariance[8] = state_cov_estimated_(4, 5);
  est_msg_pub.twist.covariance[12] = state_cov_estimated_(5, 3);
  est_msg_pub.twist.covariance[13] = state_cov_estimated_(5, 4);
  est_msg_pub.twist.covariance[14] = state_cov_estimated_(5, 5);*/

  // publish the message
  // odo_pub_.publish(est_msg_pub);
}

void ObstacleKF::PublishMessage()
{
  if (CONFIG.debug_)
    ROS_INFO("KF: PublishMessage");
  // Preform prediction based on constant velocity assumption, only position is predicted
  Eigen::Matrix<double, 6, 6> F;
  F << 1, 0, 0, CONFIG.dt_, 0, 0, 0, 1, 0, 0, CONFIG.dt_, 0, 0, 0, 1, 0, 0, CONFIG.dt_, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
      1, 0, 0, 0, 0, 0, 0, 1;
  Eigen::Matrix<double, 6, 1> state_now = state_estimated_;
  Eigen::Matrix<double, 6, 1> state_next;

  // Trajectory prediction and prepare published message
  std_msgs::Float64MultiArray msg_pub;
  msg_pub.data.resize(4 * (CONFIG.N_ + 1));

  for (int k = 0; k < CONFIG.N_ + 1; k++) // First is the current position
  {

    // std::cout << "[" << k << "]: " << state_now(0) << ", " << state_now(1) << ", " << id_ << ", " << yaw_measured_ << std::endl;
    // store into the published message
    msg_pub.data[0 + 4 * k] = state_now(0);
    msg_pub.data[1 + 4 * k] = state_now(1);
    msg_pub.data[2 + 4 * k] = id_;
    msg_pub.data[3 + 4 * k] = yaw_measured_; // yaw is assumed to be constant

    state_next = F * state_now; // predicted state
    state_now = state_next;     // set next to be now
  }

  if ((ros::Time::now() - dt_publish_).toSec() > CONFIG.dt_out_)
  {
    // publish the message
    if (CONFIG.debug_)
      ROS_INFO_STREAM("KF: Publishing message (ID = " << id_ << ")");

    pub_.publish(msg_pub);
    dt_publish_ = ros::Time::now();
  }
}
