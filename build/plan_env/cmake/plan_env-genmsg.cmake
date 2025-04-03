# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "plan_env: 3 messages, 0 services")

set(MSG_I_FLAGS "-Iplan_env:/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(plan_env_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/IdxList.msg" NAME_WE)
add_custom_target(_plan_env_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "plan_env" "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/IdxList.msg" ""
)

get_filename_component(_filename "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/ChunkStamps.msg" NAME_WE)
add_custom_target(_plan_env_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "plan_env" "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/ChunkStamps.msg" "plan_env/IdxList"
)

get_filename_component(_filename "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/ChunkData.msg" NAME_WE)
add_custom_target(_plan_env_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "plan_env" "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/ChunkData.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(plan_env
  "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/IdxList.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/plan_env
)
_generate_msg_cpp(plan_env
  "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/ChunkStamps.msg"
  "${MSG_I_FLAGS}"
  "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/IdxList.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/plan_env
)
_generate_msg_cpp(plan_env
  "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/ChunkData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/plan_env
)

### Generating Services

### Generating Module File
_generate_module_cpp(plan_env
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/plan_env
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(plan_env_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(plan_env_generate_messages plan_env_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/IdxList.msg" NAME_WE)
add_dependencies(plan_env_generate_messages_cpp _plan_env_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/ChunkStamps.msg" NAME_WE)
add_dependencies(plan_env_generate_messages_cpp _plan_env_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/ChunkData.msg" NAME_WE)
add_dependencies(plan_env_generate_messages_cpp _plan_env_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(plan_env_gencpp)
add_dependencies(plan_env_gencpp plan_env_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS plan_env_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(plan_env
  "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/IdxList.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/plan_env
)
_generate_msg_eus(plan_env
  "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/ChunkStamps.msg"
  "${MSG_I_FLAGS}"
  "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/IdxList.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/plan_env
)
_generate_msg_eus(plan_env
  "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/ChunkData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/plan_env
)

### Generating Services

### Generating Module File
_generate_module_eus(plan_env
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/plan_env
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(plan_env_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(plan_env_generate_messages plan_env_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/IdxList.msg" NAME_WE)
add_dependencies(plan_env_generate_messages_eus _plan_env_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/ChunkStamps.msg" NAME_WE)
add_dependencies(plan_env_generate_messages_eus _plan_env_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/ChunkData.msg" NAME_WE)
add_dependencies(plan_env_generate_messages_eus _plan_env_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(plan_env_geneus)
add_dependencies(plan_env_geneus plan_env_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS plan_env_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(plan_env
  "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/IdxList.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/plan_env
)
_generate_msg_lisp(plan_env
  "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/ChunkStamps.msg"
  "${MSG_I_FLAGS}"
  "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/IdxList.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/plan_env
)
_generate_msg_lisp(plan_env
  "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/ChunkData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/plan_env
)

### Generating Services

### Generating Module File
_generate_module_lisp(plan_env
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/plan_env
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(plan_env_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(plan_env_generate_messages plan_env_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/IdxList.msg" NAME_WE)
add_dependencies(plan_env_generate_messages_lisp _plan_env_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/ChunkStamps.msg" NAME_WE)
add_dependencies(plan_env_generate_messages_lisp _plan_env_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/ChunkData.msg" NAME_WE)
add_dependencies(plan_env_generate_messages_lisp _plan_env_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(plan_env_genlisp)
add_dependencies(plan_env_genlisp plan_env_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS plan_env_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(plan_env
  "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/IdxList.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/plan_env
)
_generate_msg_nodejs(plan_env
  "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/ChunkStamps.msg"
  "${MSG_I_FLAGS}"
  "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/IdxList.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/plan_env
)
_generate_msg_nodejs(plan_env
  "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/ChunkData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/plan_env
)

### Generating Services

### Generating Module File
_generate_module_nodejs(plan_env
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/plan_env
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(plan_env_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(plan_env_generate_messages plan_env_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/IdxList.msg" NAME_WE)
add_dependencies(plan_env_generate_messages_nodejs _plan_env_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/ChunkStamps.msg" NAME_WE)
add_dependencies(plan_env_generate_messages_nodejs _plan_env_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/ChunkData.msg" NAME_WE)
add_dependencies(plan_env_generate_messages_nodejs _plan_env_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(plan_env_gennodejs)
add_dependencies(plan_env_gennodejs plan_env_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS plan_env_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(plan_env
  "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/IdxList.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/plan_env
)
_generate_msg_py(plan_env
  "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/ChunkStamps.msg"
  "${MSG_I_FLAGS}"
  "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/IdxList.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/plan_env
)
_generate_msg_py(plan_env
  "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/ChunkData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/plan_env
)

### Generating Services

### Generating Module File
_generate_module_py(plan_env
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/plan_env
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(plan_env_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(plan_env_generate_messages plan_env_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/IdxList.msg" NAME_WE)
add_dependencies(plan_env_generate_messages_py _plan_env_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/ChunkStamps.msg" NAME_WE)
add_dependencies(plan_env_generate_messages_py _plan_env_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zqh/phoenixZ/racer_explore/src/RACER/swarm_exploration/plan_env/msg/ChunkData.msg" NAME_WE)
add_dependencies(plan_env_generate_messages_py _plan_env_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(plan_env_genpy)
add_dependencies(plan_env_genpy plan_env_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS plan_env_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/plan_env)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/plan_env
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(plan_env_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/plan_env)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/plan_env
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(plan_env_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/plan_env)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/plan_env
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(plan_env_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/plan_env)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/plan_env
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(plan_env_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/plan_env)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/plan_env\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/plan_env
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(plan_env_generate_messages_py std_msgs_generate_messages_py)
endif()
