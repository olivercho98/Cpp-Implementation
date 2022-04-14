cmake_minimum_required(VERSION 2.8)

project(RIFT_demo)

find_package(OpenCV REQUIRED)
message(STATUS ${OpenCV_INCLUDE_DIRS})

SET(CMAKE_CXX_COMPILER "g++")
#SET( CMAKE_BUILD_TYPE Release )
# SET( CMAKE_CXX_FLAGS "-O3 -std=c++11" )
SET( CMAKE_BUILD_TYPE Debug )
SET( CMAKE_CXX_FLAGS "-std=c++11" )


message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

include_directories(/usr/local/lib/)

include_directories(${OpenCV_INCLUDE_DIRS})
aux_source_directory(. SRC_LIST)
add_executable(RIFT_demo ${SRC_LIST})
target_link_libraries(RIFT_demo ${OpenCV_LIBS})
