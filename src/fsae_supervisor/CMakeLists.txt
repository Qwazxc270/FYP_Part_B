cmake_minimum_required(VERSION 3.0.2)
project(fsae_supervisor)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
    ResidualVector.msg
    ModePosterior.msg
    SupervisorAction.msg
    FaultFlags.msg
    MissionStatus.msg
    MpcStats.msg
    EkfState.msg
    WheelEncoder.msg
    ConeArray.msg
)

add_service_files(
  FILES
    SetFault.srv
    SetMission.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy std_msgs std_srvs geometry_msgs sensor_msgs nav_msgs
    message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  scripts/supervisor_node.py
  scripts/mission_manager_node.py
  scripts/fault_injector_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_imm_math.py)
  catkin_add_nosetests(test/test_residuals.py)
  catkin_add_nosetests(test/test_action_policy.py)
  catkin_add_nosetests(test/test_team_adapters.py)
  catkin_add_nosetests(test/test_integration_config.py)
endif()
