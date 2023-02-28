#include "obstacle_estimator/config.h"

void Config::Init()
{
  ROS_INFO("Initializing obstacle_estimator (KF)");

  ros::NodeHandle nh_config;
  ros::NodeHandle nh;

  retrieveParameter(nh, "obstacle_estimator/dt", dt_);
  retrieveParameter(nh, "obstacle_estimator/N", N_);

  retrieveParameter(nh, "obstacle_estimator/rate_out", rate_out_);
  dt_out_ = 1. / (double)rate_out_;
  retrieveParameter(nh, "obstacle_estimator/max_predictors", max_predictors_);
}