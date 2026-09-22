# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "qcar_navigation: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iqcar_navigation:/home/luke/catkin_ws/src/qcar_navigation/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(qcar_navigation_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/luke/catkin_ws/src/qcar_navigation/msg/ConeDetection.msg" NAME_WE)
add_custom_target(_qcar_navigation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "qcar_navigation" "/home/luke/catkin_ws/src/qcar_navigation/msg/ConeDetection.msg" "geometry_msgs/Point:std_msgs/Header:geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/luke/catkin_ws/src/qcar_navigation/msg/ConeDetectionArray.msg" NAME_WE)
add_custom_target(_qcar_navigation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "qcar_navigation" "/home/luke/catkin_ws/src/qcar_navigation/msg/ConeDetectionArray.msg" "geometry_msgs/Point:qcar_navigation/ConeDetection:std_msgs/Header:geometry_msgs/Vector3"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(qcar_navigation
  "/home/luke/catkin_ws/src/qcar_navigation/msg/ConeDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qcar_navigation
)
_generate_msg_cpp(qcar_navigation
  "/home/luke/catkin_ws/src/qcar_navigation/msg/ConeDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/luke/catkin_ws/src/qcar_navigation/msg/ConeDetection.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qcar_navigation
)

### Generating Services

### Generating Module File
_generate_module_cpp(qcar_navigation
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qcar_navigation
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(qcar_navigation_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(qcar_navigation_generate_messages qcar_navigation_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/luke/catkin_ws/src/qcar_navigation/msg/ConeDetection.msg" NAME_WE)
add_dependencies(qcar_navigation_generate_messages_cpp _qcar_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luke/catkin_ws/src/qcar_navigation/msg/ConeDetectionArray.msg" NAME_WE)
add_dependencies(qcar_navigation_generate_messages_cpp _qcar_navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qcar_navigation_gencpp)
add_dependencies(qcar_navigation_gencpp qcar_navigation_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qcar_navigation_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(qcar_navigation
  "/home/luke/catkin_ws/src/qcar_navigation/msg/ConeDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qcar_navigation
)
_generate_msg_eus(qcar_navigation
  "/home/luke/catkin_ws/src/qcar_navigation/msg/ConeDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/luke/catkin_ws/src/qcar_navigation/msg/ConeDetection.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qcar_navigation
)

### Generating Services

### Generating Module File
_generate_module_eus(qcar_navigation
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qcar_navigation
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(qcar_navigation_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(qcar_navigation_generate_messages qcar_navigation_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/luke/catkin_ws/src/qcar_navigation/msg/ConeDetection.msg" NAME_WE)
add_dependencies(qcar_navigation_generate_messages_eus _qcar_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luke/catkin_ws/src/qcar_navigation/msg/ConeDetectionArray.msg" NAME_WE)
add_dependencies(qcar_navigation_generate_messages_eus _qcar_navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qcar_navigation_geneus)
add_dependencies(qcar_navigation_geneus qcar_navigation_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qcar_navigation_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(qcar_navigation
  "/home/luke/catkin_ws/src/qcar_navigation/msg/ConeDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qcar_navigation
)
_generate_msg_lisp(qcar_navigation
  "/home/luke/catkin_ws/src/qcar_navigation/msg/ConeDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/luke/catkin_ws/src/qcar_navigation/msg/ConeDetection.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qcar_navigation
)

### Generating Services

### Generating Module File
_generate_module_lisp(qcar_navigation
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qcar_navigation
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(qcar_navigation_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(qcar_navigation_generate_messages qcar_navigation_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/luke/catkin_ws/src/qcar_navigation/msg/ConeDetection.msg" NAME_WE)
add_dependencies(qcar_navigation_generate_messages_lisp _qcar_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luke/catkin_ws/src/qcar_navigation/msg/ConeDetectionArray.msg" NAME_WE)
add_dependencies(qcar_navigation_generate_messages_lisp _qcar_navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qcar_navigation_genlisp)
add_dependencies(qcar_navigation_genlisp qcar_navigation_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qcar_navigation_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(qcar_navigation
  "/home/luke/catkin_ws/src/qcar_navigation/msg/ConeDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qcar_navigation
)
_generate_msg_nodejs(qcar_navigation
  "/home/luke/catkin_ws/src/qcar_navigation/msg/ConeDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/luke/catkin_ws/src/qcar_navigation/msg/ConeDetection.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qcar_navigation
)

### Generating Services

### Generating Module File
_generate_module_nodejs(qcar_navigation
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qcar_navigation
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(qcar_navigation_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(qcar_navigation_generate_messages qcar_navigation_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/luke/catkin_ws/src/qcar_navigation/msg/ConeDetection.msg" NAME_WE)
add_dependencies(qcar_navigation_generate_messages_nodejs _qcar_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luke/catkin_ws/src/qcar_navigation/msg/ConeDetectionArray.msg" NAME_WE)
add_dependencies(qcar_navigation_generate_messages_nodejs _qcar_navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qcar_navigation_gennodejs)
add_dependencies(qcar_navigation_gennodejs qcar_navigation_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qcar_navigation_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(qcar_navigation
  "/home/luke/catkin_ws/src/qcar_navigation/msg/ConeDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qcar_navigation
)
_generate_msg_py(qcar_navigation
  "/home/luke/catkin_ws/src/qcar_navigation/msg/ConeDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/luke/catkin_ws/src/qcar_navigation/msg/ConeDetection.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qcar_navigation
)

### Generating Services

### Generating Module File
_generate_module_py(qcar_navigation
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qcar_navigation
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(qcar_navigation_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(qcar_navigation_generate_messages qcar_navigation_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/luke/catkin_ws/src/qcar_navigation/msg/ConeDetection.msg" NAME_WE)
add_dependencies(qcar_navigation_generate_messages_py _qcar_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luke/catkin_ws/src/qcar_navigation/msg/ConeDetectionArray.msg" NAME_WE)
add_dependencies(qcar_navigation_generate_messages_py _qcar_navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qcar_navigation_genpy)
add_dependencies(qcar_navigation_genpy qcar_navigation_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qcar_navigation_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qcar_navigation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qcar_navigation
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(qcar_navigation_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(qcar_navigation_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qcar_navigation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qcar_navigation
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(qcar_navigation_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(qcar_navigation_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qcar_navigation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qcar_navigation
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(qcar_navigation_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(qcar_navigation_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qcar_navigation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qcar_navigation
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(qcar_navigation_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(qcar_navigation_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qcar_navigation)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qcar_navigation\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qcar_navigation
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(qcar_navigation_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(qcar_navigation_generate_messages_py std_msgs_generate_messages_py)
endif()
