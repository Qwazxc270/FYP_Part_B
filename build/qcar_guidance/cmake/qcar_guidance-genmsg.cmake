# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "qcar_guidance: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iqcar_guidance:/home/luke/catkin_ws/src/qcar_guidance/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(qcar_guidance_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/luke/catkin_ws/src/qcar_guidance/msg/TrajectoryMessage.msg" NAME_WE)
add_custom_target(_qcar_guidance_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "qcar_guidance" "/home/luke/catkin_ws/src/qcar_guidance/msg/TrajectoryMessage.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(qcar_guidance
  "/home/luke/catkin_ws/src/qcar_guidance/msg/TrajectoryMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qcar_guidance
)

### Generating Services

### Generating Module File
_generate_module_cpp(qcar_guidance
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qcar_guidance
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(qcar_guidance_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(qcar_guidance_generate_messages qcar_guidance_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/luke/catkin_ws/src/qcar_guidance/msg/TrajectoryMessage.msg" NAME_WE)
add_dependencies(qcar_guidance_generate_messages_cpp _qcar_guidance_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qcar_guidance_gencpp)
add_dependencies(qcar_guidance_gencpp qcar_guidance_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qcar_guidance_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(qcar_guidance
  "/home/luke/catkin_ws/src/qcar_guidance/msg/TrajectoryMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qcar_guidance
)

### Generating Services

### Generating Module File
_generate_module_eus(qcar_guidance
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qcar_guidance
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(qcar_guidance_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(qcar_guidance_generate_messages qcar_guidance_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/luke/catkin_ws/src/qcar_guidance/msg/TrajectoryMessage.msg" NAME_WE)
add_dependencies(qcar_guidance_generate_messages_eus _qcar_guidance_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qcar_guidance_geneus)
add_dependencies(qcar_guidance_geneus qcar_guidance_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qcar_guidance_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(qcar_guidance
  "/home/luke/catkin_ws/src/qcar_guidance/msg/TrajectoryMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qcar_guidance
)

### Generating Services

### Generating Module File
_generate_module_lisp(qcar_guidance
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qcar_guidance
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(qcar_guidance_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(qcar_guidance_generate_messages qcar_guidance_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/luke/catkin_ws/src/qcar_guidance/msg/TrajectoryMessage.msg" NAME_WE)
add_dependencies(qcar_guidance_generate_messages_lisp _qcar_guidance_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qcar_guidance_genlisp)
add_dependencies(qcar_guidance_genlisp qcar_guidance_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qcar_guidance_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(qcar_guidance
  "/home/luke/catkin_ws/src/qcar_guidance/msg/TrajectoryMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qcar_guidance
)

### Generating Services

### Generating Module File
_generate_module_nodejs(qcar_guidance
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qcar_guidance
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(qcar_guidance_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(qcar_guidance_generate_messages qcar_guidance_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/luke/catkin_ws/src/qcar_guidance/msg/TrajectoryMessage.msg" NAME_WE)
add_dependencies(qcar_guidance_generate_messages_nodejs _qcar_guidance_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qcar_guidance_gennodejs)
add_dependencies(qcar_guidance_gennodejs qcar_guidance_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qcar_guidance_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(qcar_guidance
  "/home/luke/catkin_ws/src/qcar_guidance/msg/TrajectoryMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qcar_guidance
)

### Generating Services

### Generating Module File
_generate_module_py(qcar_guidance
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qcar_guidance
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(qcar_guidance_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(qcar_guidance_generate_messages qcar_guidance_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/luke/catkin_ws/src/qcar_guidance/msg/TrajectoryMessage.msg" NAME_WE)
add_dependencies(qcar_guidance_generate_messages_py _qcar_guidance_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qcar_guidance_genpy)
add_dependencies(qcar_guidance_genpy qcar_guidance_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qcar_guidance_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qcar_guidance)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qcar_guidance
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(qcar_guidance_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qcar_guidance)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qcar_guidance
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(qcar_guidance_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qcar_guidance)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qcar_guidance
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(qcar_guidance_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qcar_guidance)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qcar_guidance
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(qcar_guidance_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qcar_guidance)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qcar_guidance\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qcar_guidance
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(qcar_guidance_generate_messages_py std_msgs_generate_messages_py)
endif()
