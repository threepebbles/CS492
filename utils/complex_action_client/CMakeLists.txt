cmake_minimum_required(VERSION 2.8.3)
project(complex_action_client)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  actionlib
  geometry_msgs
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES String_None.srv None_String.srv String_Int.srv
)

add_message_files(
  FILES EndpointState.msg
)

generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  geometry_msgs
  std_msgs
  message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/stow_ur5 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

