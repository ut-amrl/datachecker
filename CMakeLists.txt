PROJECT(datachecker)
CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0)


SET(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTORCC ON)
SET(CMAKE_AUTOUIC ON)

IF(CMAKE_VERSION VERSION_LESS "3.7.0")
    SET(CMAKE_INCLUDE_CURRENT_DIR ON)
ENDIF()

SET(CMAKE_CXX_FLAGS "-std=c++11 -march=native -Wall -Wno-unused-function -g -rdynamic")

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
SET(ROS_BUILD_STATIC_LIBS true)
SET(ROS_BUILD_SHARED_LIBS false)


SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

ROSBUILD_ADD_EXECUTABLE(bagfile_synopsis src/bagfile_synopsis.cc)
# ADD_DEPENDENCIES(${target} shared_libraries)
# TARGET_LINK_LIBRARIES(bagfile_synopsis    shared_libraries
#     ${libs})