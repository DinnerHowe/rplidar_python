cmake_minimum_required(VERSION 2.8.3)
project(rplidar_python)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
  rospy
  std_msgs
  message_generation
  geometry_msgs
  #rplidar_python_msgs
)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   rplidar_point.msg
#   rplidar_data.msg
#   rplidar_parameters.msg
# )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   #rplidar_python_msgs # Or other packages containing msgs
 )

roslaunch_add_file_check(launch)

catkin_package(
  LIBRARIES rplidar_python
  CATKIN_DEPENDS roslaunch rospy message_runtime std_msgs geometry_msgs  #rplidar_python_msgs
  DEPENDS system_lib
)



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS rplidar_python rplidar_python_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


## Mark other files for installation (e.g. launch and bag files, etc.)
 install(FILES
#   # myfile1
#   # myfile2
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )


