cmake_minimum_required(VERSION 3.0.2)
project(ur10_6dof_kinematics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(forward_kinematics src/forward_kinematics.cpp)
add_executable(inverse_kinematics src/inverse_kinematics.cpp)

target_link_libraries(forward_kinematics ${catkin_LIBRARIES})
target_link_libraries(inverse_kinematics ${catkin_LIBRARIES})
