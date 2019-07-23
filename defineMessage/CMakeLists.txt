cmake_minimum_required(VERSION 2.8.3)
project(defineMessage)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

# Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   task_reseive.msg
#   los_control.msg
# )
 add_service_files(
   FILES
   task_reseive.srv
   los_control.srv
 )

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

catkin_package(
  CATKIN_DEPENDS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        message_runtime
  DEPENDS
        Boost
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

