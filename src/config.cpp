#include "obstacle_estimator/config.h"

void Config::Init()
{
  ros::NodeHandle nh_config;
  ros::NodeHandle nh;

  retrieveParameter(nh, "obstacle_estimator/dt", dt_);
  retrieveParameter(nh, "obstacle_estimator/N", N_);

  retrieveParameter(nh, "obstacle_estimator/rate_in", rate_in_);
  retrieveParameter(nh, "obstacle_estimator/rate_out", rate_out_);
  retrieveParameter(nh, "obstacle_estimator/max_predictors", max_predictors_);
}