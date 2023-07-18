#ifndef CONFIG_OBSTACLE_H
#define CONFIG_OBSTACLE_H

#include <ros/ros.h>

#define CONFIG Config::Get()

class Config
{
public:
  // Singleton function
  static Config &Get()
  {
    static Config instance_;

    return instance_;
  }

  Config(){};
  Config(const Config &) = delete;

  ~Config(){};

  void Init();

  bool debug_;
  double dt_;
  int N_;

  double rate_out_, dt_out_;

  int max_predictors_;

public:
  /**
   * @brief Retrieve a parameter from the ROS parameter server, return false if it failed
   *
   * @tparam T Variable type
   * @param nh nodehandle
   * @param name Name of the parameter on the server
   * @param value Variable to store the read value in
   * @return true If variable exists
   * @return false If variable does not exist
   */
  template <class T>
  bool retrieveParameter(const ros::NodeHandle &nh, const std::string &name, T &value)
  {
    if (!nh.getParam(name, value))
    {
      ROS_ERROR_STREAM(" Parameter " << name << " not set on node " << ros::this_node::getName().c_str());
      return false;
    }
    else
    {
      return true;
    }
  }

  /**
   * @brief Retrieve a parameter from the ROS parameter server, otherwise use the default value
   *
   * @tparam T Variable type
   * @param nh nodehandle
   * @param name Name of the parameter on the server
   * @param value Variable to store the read value in
   * @param default_value Default value to use if the variable does not exist
   */
  template <class T>
  void retrieveParameter(const ros::NodeHandle &nh, const std::string &name, T &value, const T &default_value)
  {
    if (!nh.getParam(name, value))
    {
      ROS_WARN_STREAM("Parameter \"" << name << "\" not set on " << ros::this_node::getName().c_str()
                                     << " -> using default value: " << default_value);
      value = default_value;
    }
  }

  template <class L>
  void retrieveParameter(const ros::NodeHandle &nh, const std::string &name, std::vector<L> &value,
                         const std::vector<L> &default_value)
  {
    if (!nh.getParam(name, value))
    {
      ROS_WARN_STREAM("Parameter \"" << name << "\" not set on " << ros::this_node::getName().c_str()
                                     << " -> using default value with size: " << default_value.size());
      value = default_value;
    }
  }
};
#endif
