cmake_minimum_required(VERSION 2.8.3)
project(rosprov_msgs)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)


add_service_files(
   FILES
   LoggingService.srv
 )

generate_messages(
   DEPENDENCIES
   std_msgs
)

## Declare a catkin package
catkin_package(
   CATKIN_DEPENDS 
	message_runtime 
	std_msgs 
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)
