//
// Created by hai on 4/23/19.
//

#include <obstacle_estimator/obstacle_prediction.h>
#include <obstacle_estimator/config.h>

int main(int argc, char** argv)
{
  std::cout << "hhhhhi!" << std::endl;
  // Set up ROS
  ros::init(argc, argv, "obstacle_prediction_node");  // node name
  ros::NodeHandle nh;                                 // create a node handle
  std::cout << "super hi!" << std::endl;
  Config config_;
  config_.Init();

  std::vector<Obstacle_Prediction> predictors_;
  for (int i = 0; i < config_.max_predictors_; i++)
  {
    predictors_.emplace_back(nh, i);
  }

  ros::spin();

  return 0;
}