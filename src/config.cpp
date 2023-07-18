#include "obstacle_estimator/config.h"

void Config::Init()
{
  ros::NodeHandle nh;

  ROS_INFO("Initializing obstacle_estimator (KF)");

  retrieveParameter(nh, "obstacle_estimator/debug", debug_);
  retrieveParameter(nh, "obstacle_estimator/dt", dt_);
  retrieveParameter(nh, "obstacle_estimator/N", N_);

  retrieveParameter(nh, "obstacle_estimator/rate_out", rate_out_);
  dt_out_ = 1. / (double)rate_out_;
  retrieveParameter(nh, "obstacle_estimator/max_predictors", max_predictors_);
}