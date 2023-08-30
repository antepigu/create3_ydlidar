# Copyright(c) 2020 eaibot limited.
cmake_minimum_required(VERSION 3.5)
project(ydlidar_compensator C CXX)

##################ros2#############################################
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#Include directories
include_directories(
     ${CMAKE_SOURCE_DIR}
)
FILE(GLOB SRCS "*.cpp")
FILE(GLOB HDRS "*.h")
FILE(GLOB MATRIX_HDRS "matrix/*.hpp")

add_subdirectory(examples)

#add library
add_library(${PROJECT_NAME} ${HDRS} ${SRCS} ${MATRIX_HDRS})


