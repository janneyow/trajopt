cmake_minimum_required(VERSION 3.0.2)
project(trajopt_ros)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  trajopt
  tesseract_ros
  tesseract_planning
  octomap_ros
  pcl_conversions
  message_generation
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  tesseract_msgs
)

find_package(PCL REQUIRED COMPONENTS core features filters io segmentation surface)

add_service_files(
  FILES
  GetTrajFromTrajOpt.srv
)

generate_messages(DEPENDENCIES 
  geometry_msgs
  sensor_msgs
  tesseract_msgs
  trajectory_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM ${PCL_INCLUDE_DIRS}
)

add_executable(trajopt_action_server src/trajopt_action_server.cpp)
target_link_libraries(trajopt_action_server ${catkin_LIBRARIES} ${PCL_LIBRARIES} )
# add_dependencies(trajopt_action_server ${catkin_EXPORTED_TARGETS})
