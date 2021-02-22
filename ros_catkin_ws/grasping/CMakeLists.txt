cmake_minimum_required(VERSION 2.8.3)
project(grasping)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

#add_message_files(
 # FILES
 # mes.msg
#)

generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  INCLUDE_DIRS include
#   LIBRARIES grasping
   CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

link_directories(/home/charilaos/allsignals)

#add_executable(graspSend src/graspSend.cpp)
#target_link_libraries(graspSend ${catkin_LIBRARIES})
#add_dependencies(graspSend grasping_generate_messages_cpp)


add_executable(graspNode src/grasping_ros.cpp)
target_link_libraries(graspNode ${catkin_LIBRARIES} -llabview_dll)
add_dependencies(graspNode grasping_generate_messages_cpp)


add_executable(graspReceive src/graspReceive.cpp)
target_link_libraries(graspReceive ${catkin_LIBRARIES})
add_dependencies(graspReceive grasping_generate_messages_cpp)


