cmake_minimum_required(VERSION 3.0.2)
project(ika_plate_rct_digital_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  ika_plate_rct_digital_msgs
)

catkin_python_setup()


catkin_package(
	CATKIN_DEPENDS ika_plate_rct_digital_msgs)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS nodes/ika_plate_driver
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

