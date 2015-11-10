cmake_minimum_required(VERSION 2.8.3)
project(rplidar_python)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
  rospy
  message_generation
  geometry_msgs
)

 generate_messages(
   DEPENDENCIES
   geometry_msgs
 )

roslaunch_add_file_check(launch)

catkin_package(
  LIBRARIES rplidar_python
  CATKIN_DEPENDS roslaunch rospy message_runtime geometry_msgs  
  DEPENDS system_lib
)

 install(FILES
rplidar_gmapping_demo.launch
rplidar.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )


