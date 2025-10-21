cmake_minimum_required(VERSION 3.5)
PROJECT(ctrl1)

include($ENV{FLAIR_ROOT}/flair-src/cmake-modules/GlobalCmakeFlair.cmake)
#include($ENV{FLAIR_ROOT}/flair-dev/cmake-modules/GlobalCmakeFlair.cmake)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/simulator)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/uav)