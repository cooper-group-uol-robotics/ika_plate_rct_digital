cmake_minimum_required(VERSION 3.0.2)
project(ika_plate_rct_digital_msgs)


find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

 add_message_files(
   FILES
   IKACommand.msg
   IKAReading.msg
 )

 generate_messages(
   DEPENDENCIES
std_msgs
 )


catkin_package(
	CATKIN_DEPENDS message_runtime)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

