cmake_minimum_required(VERSION 3.0.2)
project(qcar_guidance_thomas)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  qcar_guidance
  qcar_navigation
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs visualization_msgs qcar_guidance qcar_navigation
)

catkin_install_python(PROGRAMS
  scripts/online_guidance_node.py
  scripts/fake_cone_publisher.py
  scripts/path_visualizer_node.py
  scripts/trajectory_logger_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
