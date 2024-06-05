# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "projet_multi: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iprojet_multi:/home/user/catkin_ws/src/projet_multi/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(projet_multi_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/user/catkin_ws/src/projet_multi/msg/Num.msg" NAME_WE)
add_custom_target(_projet_multi_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "projet_multi" "/home/user/catkin_ws/src/projet_multi/msg/Num.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(projet_multi
  "/home/user/catkin_ws/src/projet_multi/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/projet_multi
)

### Generating Services

### Generating Module File
_generate_module_cpp(projet_multi
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/projet_multi
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(projet_multi_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(projet_multi_generate_messages projet_multi_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/catkin_ws/src/projet_multi/msg/Num.msg" NAME_WE)
add_dependencies(projet_multi_generate_messages_cpp _projet_multi_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(projet_multi_gencpp)
add_dependencies(projet_multi_gencpp projet_multi_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS projet_multi_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(projet_multi
  "/home/user/catkin_ws/src/projet_multi/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/projet_multi
)

### Generating Services

### Generating Module File
_generate_module_eus(projet_multi
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/projet_multi
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(projet_multi_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(projet_multi_generate_messages projet_multi_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/catkin_ws/src/projet_multi/msg/Num.msg" NAME_WE)
add_dependencies(projet_multi_generate_messages_eus _projet_multi_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(projet_multi_geneus)
add_dependencies(projet_multi_geneus projet_multi_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS projet_multi_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(projet_multi
  "/home/user/catkin_ws/src/projet_multi/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/projet_multi
)

### Generating Services

### Generating Module File
_generate_module_lisp(projet_multi
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/projet_multi
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(projet_multi_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(projet_multi_generate_messages projet_multi_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/catkin_ws/src/projet_multi/msg/Num.msg" NAME_WE)
add_dependencies(projet_multi_generate_messages_lisp _projet_multi_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(projet_multi_genlisp)
add_dependencies(projet_multi_genlisp projet_multi_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS projet_multi_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(projet_multi
  "/home/user/catkin_ws/src/projet_multi/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/projet_multi
)

### Generating Services

### Generating Module File
_generate_module_nodejs(projet_multi
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/projet_multi
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(projet_multi_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(projet_multi_generate_messages projet_multi_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/catkin_ws/src/projet_multi/msg/Num.msg" NAME_WE)
add_dependencies(projet_multi_generate_messages_nodejs _projet_multi_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(projet_multi_gennodejs)
add_dependencies(projet_multi_gennodejs projet_multi_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS projet_multi_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(projet_multi
  "/home/user/catkin_ws/src/projet_multi/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/projet_multi
)

### Generating Services

### Generating Module File
_generate_module_py(projet_multi
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/projet_multi
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(projet_multi_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(projet_multi_generate_messages projet_multi_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/catkin_ws/src/projet_multi/msg/Num.msg" NAME_WE)
add_dependencies(projet_multi_generate_messages_py _projet_multi_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(projet_multi_genpy)
add_dependencies(projet_multi_genpy projet_multi_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS projet_multi_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/projet_multi)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/projet_multi
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(projet_multi_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/projet_multi)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/projet_multi
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(projet_multi_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/projet_multi)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/projet_multi
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(projet_multi_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/projet_multi)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/projet_multi
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(projet_multi_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/projet_multi)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/projet_multi\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/projet_multi
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(projet_multi_generate_messages_py std_msgs_generate_messages_py)
endif()
