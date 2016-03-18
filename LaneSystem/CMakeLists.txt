cmake_minimum_required(VERSION 2.8.3)

project(exampleOpencv)
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
endif()
find_package( OpenCV REQUIRED )

FILE(GLOB_RECURSE SRCS ./LaneDetectorSim/*.cpp ./LaneDetector/*.cpp ./LaneDetector/*.cc ./LaneDetector/*.c )

add_executable( LaneDetector64 ${SRCS} )
target_link_libraries( LaneDetector64 ${OpenCV_LIBS} )

