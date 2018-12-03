# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "patrolling_sim: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(patrolling_sim_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/dave/LogisticAgent_ws/src/patrolling_sim/srv/GoToStartPosSrv.srv" NAME_WE)
add_custom_target(_patrolling_sim_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "patrolling_sim" "/home/dave/LogisticAgent_ws/src/patrolling_sim/srv/GoToStartPosSrv.srv" "std_msgs/UInt8"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(patrolling_sim
  "/home/dave/LogisticAgent_ws/src/patrolling_sim/srv/GoToStartPosSrv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/UInt8.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/patrolling_sim
)

### Generating Module File
_generate_module_cpp(patrolling_sim
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/patrolling_sim
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(patrolling_sim_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(patrolling_sim_generate_messages patrolling_sim_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dave/LogisticAgent_ws/src/patrolling_sim/srv/GoToStartPosSrv.srv" NAME_WE)
add_dependencies(patrolling_sim_generate_messages_cpp _patrolling_sim_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(patrolling_sim_gencpp)
add_dependencies(patrolling_sim_gencpp patrolling_sim_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS patrolling_sim_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(patrolling_sim
  "/home/dave/LogisticAgent_ws/src/patrolling_sim/srv/GoToStartPosSrv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/UInt8.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/patrolling_sim
)

### Generating Module File
_generate_module_eus(patrolling_sim
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/patrolling_sim
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(patrolling_sim_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(patrolling_sim_generate_messages patrolling_sim_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dave/LogisticAgent_ws/src/patrolling_sim/srv/GoToStartPosSrv.srv" NAME_WE)
add_dependencies(patrolling_sim_generate_messages_eus _patrolling_sim_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(patrolling_sim_geneus)
add_dependencies(patrolling_sim_geneus patrolling_sim_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS patrolling_sim_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(patrolling_sim
  "/home/dave/LogisticAgent_ws/src/patrolling_sim/srv/GoToStartPosSrv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/UInt8.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/patrolling_sim
)

### Generating Module File
_generate_module_lisp(patrolling_sim
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/patrolling_sim
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(patrolling_sim_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(patrolling_sim_generate_messages patrolling_sim_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dave/LogisticAgent_ws/src/patrolling_sim/srv/GoToStartPosSrv.srv" NAME_WE)
add_dependencies(patrolling_sim_generate_messages_lisp _patrolling_sim_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(patrolling_sim_genlisp)
add_dependencies(patrolling_sim_genlisp patrolling_sim_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS patrolling_sim_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(patrolling_sim
  "/home/dave/LogisticAgent_ws/src/patrolling_sim/srv/GoToStartPosSrv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/UInt8.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/patrolling_sim
)

### Generating Module File
_generate_module_nodejs(patrolling_sim
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/patrolling_sim
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(patrolling_sim_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(patrolling_sim_generate_messages patrolling_sim_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dave/LogisticAgent_ws/src/patrolling_sim/srv/GoToStartPosSrv.srv" NAME_WE)
add_dependencies(patrolling_sim_generate_messages_nodejs _patrolling_sim_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(patrolling_sim_gennodejs)
add_dependencies(patrolling_sim_gennodejs patrolling_sim_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS patrolling_sim_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(patrolling_sim
  "/home/dave/LogisticAgent_ws/src/patrolling_sim/srv/GoToStartPosSrv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/UInt8.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/patrolling_sim
)

### Generating Module File
_generate_module_py(patrolling_sim
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/patrolling_sim
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(patrolling_sim_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(patrolling_sim_generate_messages patrolling_sim_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dave/LogisticAgent_ws/src/patrolling_sim/srv/GoToStartPosSrv.srv" NAME_WE)
add_dependencies(patrolling_sim_generate_messages_py _patrolling_sim_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(patrolling_sim_genpy)
add_dependencies(patrolling_sim_genpy patrolling_sim_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS patrolling_sim_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/patrolling_sim)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/patrolling_sim
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(patrolling_sim_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/patrolling_sim)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/patrolling_sim
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(patrolling_sim_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/patrolling_sim)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/patrolling_sim
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(patrolling_sim_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/patrolling_sim)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/patrolling_sim
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(patrolling_sim_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/patrolling_sim)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/patrolling_sim\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/patrolling_sim
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(patrolling_sim_generate_messages_py std_msgs_generate_messages_py)
endif()
