cmake_minimum_required(VERSION 2.8.3)
project(spido_riding)

find_package(catkin REQUIRED COMPONENTS 
roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation)



catkin_package(
CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})





#add_executable(spido_riding_node src/spido_riding.cpp)
#target_link_libraries(spido_riding_node ${catkin_LIBRARIES})

#add_executable(spido_tracking_line_node src/spido_tracking_li#ne.cpp)
#target_link_libraries(spido_tracking_line_node ${catkin_LIBRAR#""IES})


#add_executable(spido_tracking_circle_node src/spido_tracking_circle.cpp#)
#target_link_libraries(spido_tracking_circle_node ${catkin_LIBRARIES})

#add_executable(kinematic_single_steering src/kinematic_single_steering.cpp)
#target_link_libraries(kinematic_single_steering ${catkin_LIBRARIES})

