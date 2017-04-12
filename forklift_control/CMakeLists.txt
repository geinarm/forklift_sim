cmake_minimum_required(VERSION 2.8.3)
project(forklift_control)

find_package(catkin REQUIRED genmsg actionlib_msgs geometry_msgs actionlib)
add_action_files(DIRECTORY action FILES FollowPath.action)
generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs)

catkin_package()

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
