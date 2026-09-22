cmake_minimum_required(VERSION 3.0.2)
project(qcar_guidance_thomas)

find_package(catkin REQUIRED COMPONENTS
  rospy std_msgs geometry_msgs nav_msgs visualization_msgs
  qcar_guidance qcar_navigation
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs visualization_msgs
                 qcar_guidance qcar_navigation
)

# ROS nodes
catkin_install_python(PROGRAMS
  scripts/online_guidance_node.py
  scripts/fake_track_sim_node.py
  scripts/path_visualizer_node.py
  scripts/trajectory_logger_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# plain Python modules imported by the nodes (no ROS inside)
install(FILES scripts/guidance_core.py scripts/track_sim.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
