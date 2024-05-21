#ifndef NAVIGATION_GOAL_H
#define NAVIGATION_GOAL_H

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "navigation_with_bt.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <tf2/LinearMath/Quaternion.h>

class NavigationGoal : public rclcpp::Node
{
public:
  explicit NavigationGoal(const std::string &node_name);
  void setUP();
  
private:
  void createBehaviorTree();
  void updateBehaviorTree();

  rclcpp::TimerBase::SharedPtr m_timer;
  BT::Tree m_tree;
  std::string m_my_arg;
};

#endif
