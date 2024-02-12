cmake_minimum_required(VERSION 3.0.2)
project(ik_solver_msgs)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(ROS_VERSION $ENV{ROS_VERSION})
if(NOT((${ROS_VERSION} EQUAL 1) OR (${ROS_VERSION} EQUAL 2)))
  message(FATAL_ERROR "No version of ROS found!")
endif()

list(APPEND ik_msgs Configuration.msg
                    CollisionResult.msg
                    IkSolution.msg
                    IkTarget.msg
                    )

list(APPEND ik_srvs GetIk.srv
                    GetFk.srv
                    GetIkArray.srv
                    GetFkArray.srv
                    CollisionChecking.srv
                    NeighbourhoodPyramidIk.srv
                    GetBound.srv
                    )

## ROS 1
if(${ROS_VERSION} EQUAL 1)
  find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    std_msgs
    moveit_msgs
    message_generation
  )

  add_message_files(
    FILES
    ${ik_msgs}
  )

  add_service_files(
    FILES
    ${ik_srvs}
  )

  generate_messages(
    DEPENDENCIES
    geometry_msgs std_msgs moveit_msgs
  )

  catkin_package(
  )

## ROS 2
elseif(${ROS_VERSION} EQUAL 2)
  list(APPEND ik_msgs_2)
  foreach(msg ${ik_msgs})
    string(PREPEND ${msg} "msg/")
    list(APPEND ik_msgs_2 msg)
  endforeach()
  list(APPEND ik_srvs_2)
  foreach(srv ${ik_srvs})
    string(PREPEND ${srv} "srv/")
    list(APPEND ik_srvs_2 srv)
  endforeach()

  find_package(ament_cmake REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(std_msgs      REQUIRED)
  find_package(moveit_msgs   REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    ${ik_msgs_2}
    ${ik_srvs_2}
    DEPENDENCIES geometry_msgs std_msgs moveit_msgs
  )

  ament_package()
endif()
