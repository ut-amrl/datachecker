cmake_minimum_required(VERSION 3.0.2)
project(datachecker)

add_compile_options(-std=c++11)

find_package(yaml-cpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  ouster_ros std_msgs sensor_msgs roscpp rosbag rosbag
  # other ROS packages you use
)

catkin_package(
  INCLUDE_DIRS
    ${_ouster_ros_INCLUDE_DIRS}
    # ${YAML_CPP_INCLUDE_DIRS}
  LIBRARIES 
    datachecker
    ouster_ros
  CATKIN_DEPENDS 
    ouster_ros
    roscpp
    rosbag
    message_runtime
    std_msgs
    sensor_msgs
    geometry_msgs
  # other ROS packages
  # DEPENDS system_lib
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/datachecker.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)