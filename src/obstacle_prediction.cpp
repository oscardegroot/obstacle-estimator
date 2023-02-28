//
// Created by hai on 22/4/19.
//

#include <obstacle_estimator/obstacle_prediction.h>

// Constructor:  this will get called whenever an instance of this class is created
Obstacle_Prediction::Obstacle_Prediction(ros::NodeHandle nh, int id)
{
  ROS_INFO_STREAM("Initializing KF for obstacle " << id);

  nh_ = nh;
  id_ = id + 1;  // Because mocap starts at "1"
  config_.Init();

  // Initialization subscriber and publisher
  initializeSubscribers();
  initializePublishers();

  // Initialization the state covariance, this is important for starting the Kalman filter
  double cov_pos = 1 ^ 2;
  double cov_vel = 1 ^ 2;
  state_cov_estimated_.setZero();
  state_cov_estimated_(0, 0) = cov_pos;
  state_cov_estimated_(1, 1) = cov_pos;
  state_cov_estimated_(2, 2) = cov_pos;
  state_cov_estimated_(3, 3) = cov_vel;
  state_cov_estimated_(4, 4) = cov_vel;
  state_cov_estimated_(5, 5) = cov_vel;

  // Other initialization
  pos_measured_.setZero();
  state_estimated_.setZero();

  time_stamp_ = ros::Time::now();
  time_stamp_previous_ = ros::Time::now();
  dt_ = config_.dt_;
  dt_publish_ = ros::Time::now().toSec();

  publish_counter_ = 0;
}

// Set up subscribers
void Obstacle_Prediction::initializeSubscribers()
{
  ROS_INFO("Initializing subscribers");
  sub_ = nh_.subscribe("/mocap_obstacle" + std::to_string(id_) + "/pose", 1, &Obstacle_Prediction::subscriberCallback,
                       this);
}

// Set up publisher
void Obstacle_Prediction::initializePublishers()
{
  ROS_INFO("Initializing publishers");
  pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/obstacle" + std::to_string(id_) + "/path_prediction", 1, true);
  odo_pub_ = nh_.advertise<nav_msgs::Odometry>("/obstacle" + std::to_string(id_) + "/state_estimation", 1, true);
}

// Subscriber callback function
void Obstacle_Prediction::subscriberCallback(const geometry_msgs::PoseStamped& msg)
{
  // get measured position
  pos_measured_(0) = msg.pose.position.x;
  pos_measured_(1) = msg.pose.position.y;
  pos_measured_(2) = msg.pose.position.z * 0.5;  // since a hat is wearing

  // current time stamp of the message
  time_stamp_ = msg.header.stamp;

  // time difference. If using the node_rate to derive, then comment the following lines
  dt_ = (time_stamp_ - time_stamp_previous_).toSec();  // sec
  // dt_publish_ += dt_;

  // perform Kalman Filtering
  // matrix of state transition model
  Eigen::Matrix<double, 6, 6> A;
  A << 1, 0, 0, dt_, 0, 0, 0, 1, 0, 0, dt_, 0, 0, 0, 1, 0, 0, dt_, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;
  Eigen::Matrix<double, 6, 3> B;
  double dt_2 = 0.5 * dt_ * dt_;
  B << dt_2, 0, 0, 0, dt_2, 0, 0, 0, dt_2, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  // matrix of observation model
  Eigen::Matrix<double, 3, 6> H;
  H << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;

  // noise matrix
  double R_pos = 10E-4;
  Eigen::Matrix<double, 3, 3> R;  // observation noise covariance
  R << R_pos, 0, 0, 0, R_pos, 0, 0, 0, R_pos;
  double Q_acc = 1000;  // mean of noisy acceleration
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
  Eigen::Vector3d pos_residual;  // pos estimation residual
  pos_residual = pos_measured_ - H * state_estimated_;
  Eigen::Matrix<double, 3, 3> S;
  S = H * state_cov_estimated_ * H.transpose() + R;
  Eigen::Matrix<double, 6, 3> K;  // the Kalman gain matrix
  K = (state_cov_estimated_ * H.transpose()) * S.inverse();
  // new estimated state
  state_estimated_ = state_estimated_ + K * pos_residual;
  // new covariance
  Eigen::Matrix<double, 6, 6> I;
  I.setIdentity();  // I is an identity matrix
  state_cov_estimated_ = (I - K * H) * state_cov_estimated_;

  // prepare published state estimation message
  nav_msgs::Odometry est_msg_pub;   // published obstacle state estimation
  est_msg_pub.header = msg.header;  // save the header information
  est_msg_pub.pose.pose.orientation = msg.pose.orientation;
  est_msg_pub.pose.pose.position.x = state_estimated_(0);  // save the estimated position
  est_msg_pub.pose.pose.position.y = state_estimated_(1);
  est_msg_pub.pose.pose.position.z = state_estimated_(2);
  est_msg_pub.twist.twist.linear.x = state_estimated_(3);  // save the estimated velocity
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
  est_msg_pub.twist.covariance[14] = state_cov_estimated_(5, 5);

  // publish the message
  odo_pub_.publish(est_msg_pub);

  // compute the current yaw of the obstacle
  tf::Quaternion quat_measured;
  Eigen::Matrix<double, 3, 1> euler_measured;
  tf::quaternionMsgToTF(msg.pose.orientation, quat_measured);
  tf::Matrix3x3(quat_measured).getRPY(euler_measured(0), euler_measured(1), euler_measured(2));
  double yaw_measured = euler_measured(2);

  // Trajectory prediction and prepare published message
  std_msgs::Float64MultiArray msg_pub;
  msg_pub.data.resize(4 * config_.N_);
  // Preform prediction based on constant velocity assumption, only position is predicted
  Eigen::Matrix<double, 6, 6> F;
  F << 1, 0, 0, config_.dt_, 0, 0, 0, 1, 0, 0, config_.dt_, 0, 0, 0, 1, 0, 0, config_.dt_, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
      1, 0, 0, 0, 0, 0, 0, 1;
  Eigen::Matrix<double, 6, 1> state_now;
  Eigen::Matrix<double, 6, 1> state_next;
  state_now = state_estimated_;
  for (int i = 0; i < config_.N_; i++)
  {
    // store into the published message
    msg_pub.data[0 + 4 * i] = state_now[0];
    msg_pub.data[1 + 4 * i] = state_now[1];
    msg_pub.data[2 + 4 * i] = id_;
    msg_pub.data[3 + 4 * i] = yaw_measured;  // yaw is assumed to be constant
    // predicted state
    state_next = F * state_now;
    // set next to be now
    state_now = state_next;
  }

  // publish the message

  if ((ros::Time::now().toSec() - dt_publish_) > config_.dt_out_)
  {
    // std::cout << "publishing KF for ID = " << id_ << " (x = " << state_now[0] << ", " << state_now[1] << ")"
    //           << std::endl;
    pub_.publish(msg_pub);
    dt_publish_ = ros::Time::now().toSec();
  }

  publish_counter_++;
  // set time
  time_stamp_previous_ = time_stamp_;
}
