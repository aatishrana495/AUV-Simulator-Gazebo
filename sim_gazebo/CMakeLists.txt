cmake_minimum_required(VERSION 2.8.3)
project(sim_gazebo)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
  message_generation
  gazebo_msgs
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

add_library(buoyancy_plugin src/buoyancy_plugin.cpp)
target_link_libraries(buoyancy_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(auv_plugin src/auv_plugin.cpp)
target_link_libraries(auv_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(data_publisher src/data_publisher.cpp)
add_dependencies(data_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(data_publisher ${catkin_LIBRARIES})


add_message_files(
   FILES
   ThrusterSpeeds.msg
   Combined.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
DEPENDS gazebo_ros roscpp rospy std_msgs message_runtime
)
