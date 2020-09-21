cmake_minimum_required(VERSION 2.8.3)

project(rr_viz)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  move_base_msgs
  rospy
  std_msgs
  rviz_textured_sphere
  message_generation
)
catkin_python_setup()


add_message_files(
  FILES
  RRTaskWaypoint.msg
  RRTaskWaypointList.msg
)

add_service_files(
  FILES
  BuildBT.srv
)

generate_messages(DEPENDENCIES geometry_msgs std_msgs rr_viz)


catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS actionlib geometry_msgs move_base_msgs rospy std_msgs message_runtime
)

#catkin_install_python(
#  PROGRAMS src/rr_viz
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

# Install launch files
#install(DIRECTORY launch/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#)
