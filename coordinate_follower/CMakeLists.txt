cmake_minimum_required(VERSION 3.8)
project(coordinate_bot)

find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Action interface generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GoToCoordinate.action"
  DEPENDENCIES builtin_interfaces std_msgs
)

# Install Python scripts
install(PROGRAMS
  scripts/goal_follower_server.py
  scripts/goal_sender_client.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
