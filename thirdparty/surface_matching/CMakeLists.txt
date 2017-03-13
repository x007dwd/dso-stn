SET(PROJECT_NAME surface_matching)

PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET (CMAKE_VERBOSE_MAKEFILE ON)

#set(the_description "3D point features")
#ocv_define_module(surface_matching opencv_core opencv_flann WRAP python)
include_directories(
  ${PROJECT_SOURCE_DIR}/thirdparty/surface_matching/include
)
add_library(sm ${PROJECT_SOURCE_DIR}/thirdparty/surface_matching/src/*})
