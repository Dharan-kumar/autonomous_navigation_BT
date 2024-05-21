import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
  pkg_navigation_goal = get_package_share_directory('navigation_goal')

  # Declare a command-line argument named "my_arg" with a default value
  declare_my_arg = DeclareLaunchArgument(
    'my_arg',
    default_value='default_value',
    description='My custom command-line argument'
    )

    # Create a Node with a parameter that uses the command-line argument
  nav_goal = Node(
      package="navigation_goal",
      executable="navigation_goal",
      name="navigation_node",
      parameters=[
        {"location_file": os.path.join(pkg_navigation_goal, "config", "target_poses.yaml")},
        {"my_arg": LaunchConfiguration('my_arg')}  # Use the command-line argument as a parameter 
      ]
  )

  ld = LaunchDescription()

  #Add the command-line argument declaration to the launch description
  ld.add_action(declare_my_arg)


  # Add the commands to the launch description
  ld.add_action(nav_goal)

  return ld