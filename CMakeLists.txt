cmake_minimum_required(VERSION 3.0.2)
project(ik_solver_msgs)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  moveit_msgs
  message_generation
)

add_message_files(
  FILES
  Configuration.msg
  CollisionResult.msg
  IkSolution.msg
)

add_service_files(
  FILES
  GetIk.srv
  GetIkArray.srv
  CollisionChecking.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs std_msgs moveit_msgs
)

catkin_package(
)
