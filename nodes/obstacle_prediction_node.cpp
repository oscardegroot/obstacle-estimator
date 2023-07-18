//
// Created by hai on 4/23/19.
//

#include <obstacle_estimator/obstacle_prediction.h>
#include <obstacle_estimator/config.h>

std::vector<std::unique_ptr<ObstacleKF>> predictors_;

int main(int argc, char **argv)
{
  // Set up ROS
  ros::init(argc, argv, "obstacle_prediction_node"); // node name

  ros::NodeHandle nh; // create a node handle

  CONFIG.Init();

  for (int i = 0; i < CONFIG.max_predictors_; i++)
  {
    predictors_.emplace_back(nullptr);
    predictors_.back().reset(new ObstacleKF(i));
  }

  ros::spin();
  return 0;
}