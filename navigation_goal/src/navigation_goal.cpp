#include "navigation_goal.hpp"

using namespace std::chrono_literals;

const std::string bt_xml_dir = ament_index_cpp::get_package_share_directory("navigation_goal") + "/behavior_tree_xml";

NavigationGoal::NavigationGoal(const std::string &nodeName) : Node(nodeName)
{
  RCLCPP_INFO(get_logger(), "NavigationGoal Constructor Is Invoked");
  this->declare_parameter("location_file","none");

  // Retrieve the 'my_arg' argument and store it in the private member variable
  m_my_arg = this->declare_parameter<std::string>("my_arg", "default_value");

  // Use the 'my_arg' value as needed in your node's logic
  RCLCPP_INFO(get_logger(), "Received 'my_arg' value: %s", m_my_arg.c_str());

  RCLCPP_INFO(get_logger(), "Init done");
}

void NavigationGoal::setUP()
{
  RCLCPP_INFO(get_logger(), "Setting up The Behavior Tree");
  createBehaviorTree();
  RCLCPP_INFO(get_logger(), "Behavior Tree Is Created");

  const auto timer_period = 500ms;
  m_timer = this->create_wall_timer( timer_period, std::bind(&NavigationGoal::updateBehaviorTree, this));

  rclcpp::spin(shared_from_this());
  rclcpp::shutdown();
}

void NavigationGoal::createBehaviorTree()
{
  BT::BehaviorTreeFactory factory;

  // register bt node
  BT::NodeBuilder builder = [=](const std::string &name, const BT::NodeConfiguration &config)
  {
    return std::make_unique<GoToPose>(name, m_my_arg, config, shared_from_this());
  };

  factory.registerBuilder<GoToPose>("GoToPose", builder);

  RCLCPP_INFO(get_logger(), bt_xml_dir.c_str());

  m_tree = factory.createTreeFromFile(bt_xml_dir + "/tree.xml");
}

void NavigationGoal::updateBehaviorTree()
{
  BT::NodeStatus tree_status = m_tree.tickRoot();

  if (tree_status == BT::NodeStatus::RUNNING)
  {
    return;
  }
  else if (tree_status == BT::NodeStatus::SUCCESS)
  {
    RCLCPP_INFO(this->get_logger(), "Navigation Is Finished");
    m_timer->cancel();
  }
  else if (tree_status == BT::NodeStatus::FAILURE)
  {
    RCLCPP_INFO(this->get_logger(), "Navigation Failed");
    m_timer->cancel();
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigationGoal>("navigation_node");
  node->setUP();

  return 0;
}
