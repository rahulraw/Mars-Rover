# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "cameras: 1 messages, 0 services")

set(MSG_I_FLAGS "-Icameras:/home/sean/FYDP/uwrobotics.uwmrt/Software/Workspace/src/cameras/msg;-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(cameras_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(cameras
  "/home/sean/FYDP/uwrobotics.uwmrt/Software/Workspace/src/cameras/msg/video.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cameras
)

### Generating Services

### Generating Module File
_generate_module_cpp(cameras
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cameras
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(cameras_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(cameras_generate_messages cameras_generate_messages_cpp)

# target for backward compatibility
add_custom_target(cameras_gencpp)
add_dependencies(cameras_gencpp cameras_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cameras_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(cameras
  "/home/sean/FYDP/uwrobotics.uwmrt/Software/Workspace/src/cameras/msg/video.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cameras
)

### Generating Services

### Generating Module File
_generate_module_lisp(cameras
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cameras
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(cameras_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(cameras_generate_messages cameras_generate_messages_lisp)

# target for backward compatibility
add_custom_target(cameras_genlisp)
add_dependencies(cameras_genlisp cameras_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cameras_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(cameras
  "/home/sean/FYDP/uwrobotics.uwmrt/Software/Workspace/src/cameras/msg/video.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cameras
)

### Generating Services

### Generating Module File
_generate_module_py(cameras
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cameras
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(cameras_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(cameras_generate_messages cameras_generate_messages_py)

# target for backward compatibility
add_custom_target(cameras_genpy)
add_dependencies(cameras_genpy cameras_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cameras_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cameras)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cameras
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(cameras_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cameras)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cameras
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(cameras_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cameras)
  install(CODE "execute_process(COMMAND \"/usr/local/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cameras\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cameras
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(cameras_generate_messages_py std_msgs_generate_messages_py)
