#include "navigation_with_bt.hpp"
#include "yaml-cpp/yaml.h"
#include <string>


GoToPose::GoToPose(const std::string &name,const std::string &my_arg,
                   const BT::NodeConfiguration &config,
                   rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), m_node_ptr(node_ptr)
{
  RCLCPP_INFO(m_node_ptr->get_logger(), "GoToPose Constructor Is Invoked");
  m_action_client_ptr = rclcpp_action::create_client<NavigateToPose>(m_node_ptr, "/navigate_to_pose");
  m_subPtr = m_node_ptr->create_subscription<geometry_msgs::msg::PointStamped>(
      "clicked_point", 10, std::bind(&GoToPose::positionCallback, this, std::placeholders::_1));
  done_flag_ = false;
  this->my_arg = my_arg;
  if(my_arg == "navigate")
   check_flag = true;
  
};

void GoToPose::getCommandLineData()
{
  RCLCPP_INFO(m_node_ptr->get_logger(), "Feteching The Inputs");
  rclcpp::NodeOptions op;
  auto argum = op.arguments();
  std::string cmd_data;

}

void GoToPose::positionCallback(const geometry_msgs::msg::PointStamped::SharedPtr poistion_data)
{
  //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  m_publish_point = *poistion_data;

  RCLCPP_INFO(m_node_ptr->get_logger(), "Received data %f %f %f", m_publish_point.point.x, m_publish_point.point.y, m_publish_point.point.z);
  
  if(my_arg == "record")
  {
      const std::string file_path_ = "/home/dharan/robotspace_ws/src/config/publish_point.yaml";
      // Load existing YAML data from the file (if any)
      YAML::Node yaml_node;
      try
      {
        std::ifstream ifs(file_path_);
        if (ifs)
        {
          yaml_node = YAML::LoadFile(file_path_);
        }
      }
      catch (const YAML::ParserException & e)
      {
        RCLCPP_ERROR(m_node_ptr->get_logger(), "Error loading existing YAML data: %s", e.what());
      }

      // Create a YAML node to represent the received PointStamped message
      YAML::Node point_node;
      point_node["header"]["frame_id"] = m_publish_point.header.frame_id;
      point_node["header"]["stamp"]["sec"] = m_publish_point.header.stamp.sec;
      point_node["header"]["stamp"]["nanosec"] = m_publish_point.header.stamp.nanosec;
      point_node["point"]["x"] = m_publish_point.point.x;
      point_node["point"]["y"] = m_publish_point.point.y;
      point_node["point"]["z"] = m_publish_point.point.z;

      // Append the new point node to the existing YAML data
      yaml_node["points"].push_back(point_node);

      // Write the updated YAML data back to the file
      try
      {
        std::ofstream ofs(file_path_);
        ofs << yaml_node;
        RCLCPP_INFO(m_node_ptr->get_logger(), "Data appended to yaml file");
      }
      catch (const YAML::EmitterException & e)
      {
        RCLCPP_ERROR(m_node_ptr->get_logger(), "Error writing YAML data to file: %s", e.what());
      }
    }
  }
    // // Define a YAML emitter
    // YAML::Emitter emitter;

    // // Create a YAML node to represent the PointStamped message
    // YAML::Node yaml_node;
    // yaml_node["header"]["frame_id"] = m_publish_point.header.frame_id;
    // yaml_node["header"]["stamp"]["sec"] = m_publish_point.header.stamp.sec;
    // yaml_node["header"]["stamp"]["nanosec"] = m_publish_point.header.stamp.nanosec;
    // yaml_node["point"]["x"] = m_publish_point.point.x;
    // yaml_node["point"]["y"] = m_publish_point.point.y;
    // yaml_node["point"]["z"] = m_publish_point.point.z;

    // // Specify the file path where you want to save the YAML file
    // const std::string file_path = "/home/dharan/robotspace_ws/src/config/publish_point.yaml";

    // // Write the YAML data to the file
    // std::ofstream ofs(file_path);
    // ofs << yaml_node;
    // RCLCPP_INFO(m_node_ptr->get_logger(), "Data written into yaml file");
   //}
 //}

BT::PortsList GoToPose::providedPorts()
{
  return {BT::InputPort<std::string>("loc")};
}

BT::NodeStatus GoToPose::onStart()
{
  // Get location key from port and read YAML file
  BT::Optional<std::string> loc = getInput<std::string>("loc");
  const std::string location_file = m_node_ptr->get_parameter("location_file").as_string();

  // YAML::Node locations = YAML::LoadFile(location_file);

  // std::vector<float> pose = locations[loc.value()].as<std::vector<float>>();

  // setup action client
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback = std::bind(&GoToPose::navPoseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(&GoToPose::navFeedback, this, std::placeholders::_1, std::placeholders::_2);

  auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
  //goal_msg = NavigateToPose::Goal();

  if(check_flag)
  {
    RCLCPP_INFO(m_node_ptr->get_logger(),"Reading Data From Yaml");
    // Specify the file path where the YAML file is located
    const std::string file_path = "/home/dharan/robotspace_ws/src/config/publish_point.yaml";

    // Load and parse the YAML file
    YAML::Node yaml_node = YAML::LoadFile(file_path);

    if (yaml_node["points"].IsSequence())
    {
      const auto &points = yaml_node["points"];
      for (const auto &point : points)
      {
        if (point["point"]["x"] && point["point"]["y"])
        {
          // Extract the goal coordinates from the YAML data
          double x = point["point"]["x"].as<double>();
          double y = point["point"]["y"].as<double>();
          //double z = point["point"]["z"].as<double>();

          goal_msg.pose.header.frame_id = "map";
          goal_msg.pose.pose.position.x = x;//pose[0];
          goal_msg.pose.pose.position.y = y;//pose[1];

          tf2::Quaternion q;
          q.setRPY(0, 0, 0.1);
          //q.setRPY(0, 0, pose[2]);
          q.normalize(); 
          goal_msg.pose.pose.orientation = tf2::toMsg(q);

          // send pose
          done_flag_ = false;
          g_msg = goal_msg;
          m_action_client_ptr->async_send_goal(goal_msg, send_goal_options);
          RCLCPP_INFO(m_node_ptr->get_logger(), "Sent Goal to Nav2 Frame Work\n");
          RCLCPP_INFO(m_node_ptr->get_logger(),"Reading Data From Yaml");
          return BT::NodeStatus::RUNNING;
        }
        return BT::NodeStatus::RUNNING;
      }
    }
    else
    {
      RCLCPP_INFO(m_node_ptr->get_logger(), "No points found in the YAML file");
      return BT::NodeStatus::FAILURE;
    }
  }
  else
  {
    RCLCPP_INFO(m_node_ptr->get_logger(), "Wrong Input Argument %s", my_arg.c_str());
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus GoToPose::onRunning()
{
  if (done_flag_)
  {
    RCLCPP_INFO(m_node_ptr->get_logger(), "Target Goal reached\n");
    return BT::NodeStatus::SUCCESS;  
  }
  else
  {
    return BT::NodeStatus::RUNNING;
  }
}

void GoToPose::navPoseCallback(const GoalHandleNav::WrappedResult &result)
{
  if (result.result)
  {
    done_flag_ = true;
  }
}

void GoToPose::navFeedback(GoalHandleNav::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  // float dx = feedback->current_pose.pose.position.x - g_msg.pose.pose.position.x;
  // float dy = feedback->current_pose.pose.position.x - g_msg.pose.pose.position.x;
  // float dz = feedback->current_pose.pose.position.x - g_msg.pose.pose.position.x;

  // auto distance_left = std::sqrt(dx*dx + dy*dy + dz*dz);
  // RCLCPP_INFO(m_node_ptr->get_logger(),"My Custom Robot Distance Left : %f",distance_left);

  RCLCPP_INFO(m_node_ptr->get_logger(),"The Remaining Distance Left Is: %f",feedback->distance_remaining);
  RCLCPP_INFO(m_node_ptr->get_logger(),"The Number Of Recoveries: %d",feedback->number_of_recoveries);
  
}
