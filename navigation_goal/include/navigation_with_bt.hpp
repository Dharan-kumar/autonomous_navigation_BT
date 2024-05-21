#ifndef NAVIGATION_WITH_BT_H
#define NAVIGATION_WITH_BT_H

#include<iostream>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <bits/stdc++.h>

#include"geometry_msgs/msg/point_stamped.hpp"

//#include <geometry_msgs::msg::PointStamped.h>

class GoToPose : public BT::StatefulActionNode
{
public:
  GoToPose(const std::string &name,const std::string &my_arg,
           const BT::NodeConfiguration &config,
           rclcpp::Node::SharedPtr node_ptr);

  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  static BT::PortsList providedPorts();
  void getCommandLineData();

private:
  nav2_msgs::action::NavigateToPose::Goal g_msg;
  rclcpp::Node::SharedPtr m_node_ptr;
  rclcpp_action::Client<NavigateToPose>::SharedPtr m_action_client_ptr;
  bool done_flag_;
  bool check_flag;
  std::string my_arg;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr m_subPtr;

  geometry_msgs::msg::PointStamped m_publish_point;

  // Method overrides
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override{};

  // Action Client callback
  void navPoseCallback(const GoalHandleNav::WrappedResult &result);
  void navFeedback(GoalHandleNav::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback);

  // Position marker callback
  void positionCallback(const geometry_msgs::msg::PointStamped::SharedPtr poistion_data);
};

#endif

