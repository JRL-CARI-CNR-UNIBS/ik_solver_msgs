cmake_minimum_required(VERSION 3.8)
project(ik_solver_msgs)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(ROS_VERSION $ENV{ROS_VERSION})
if(ROS_VERSION STREQUAL "")
  message(FATAL_ERROR "No supported version of ROS found!")
endif()
if(NOT((ROS_VERSION EQUAL 1) OR (ROS_VERSION EQUAL 2)))
  message(FATAL_ERROR "No supported version of ROS found!")
endif()

## ROS 1
if(ROS_VERSION EQUAL 1)
  message("Compiling for ROS_VERSION == 1")
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
    IkTarget.msg
  )

  add_service_files(
    FILES
    GetIk.srv
    GetFk.srv
    GetIkArray.srv
    GetFkArray.srv
    CollisionChecking.srv
    NeighbourhoodPyramidIk.srv
    GetBound.srv
  )

  generate_messages(
    DEPENDENCIES
    geometry_msgs std_msgs moveit_msgs
  )

  catkin_package(
  )

## ROS 2
elseif(ROS_VERSION EQUAL 2)
message("Compiling for ROS_VERSION == 2")
  find_package(ament_cmake REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(std_msgs      REQUIRED)
  find_package(moveit_msgs   REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Configuration.msg"
    "msg/ros2/CollisionResult.msg"
    "msg/IkSolution.msg"
    "msg/IkTarget.msg"
    "srv/GetIk.srv"
    "srv/GetFk.srv"
    "srv/GetIkArray.srv"
    "srv/GetFkArray.srv"
    "srv/CollisionChecking.srv"
    "srv/NeighbourhoodPyramidIk.srv"
    "srv/GetBound.srv"
    DEPENDENCIES geometry_msgs std_msgs moveit_msgs
  )

  ament_package()
endif()
