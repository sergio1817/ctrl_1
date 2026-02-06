PROJECT(ctrl1_simulator)
cmake_minimum_required(VERSION 3.5)

SET(FLAIR_USE_SIMULATOR TRUE)
SET(FLAIR_USE_SIMULATOR_GL TRUE)

include($ENV{FLAIR_ROOT}/flair-src/cmake-modules/GlobalCmakeFlair.cmake)
#include($ENV{FLAIR_ROOT}/flair-dev/cmake-modules/GlobalCmakeFlair.cmake)

SET(SRC_FILES
	./src/main.cpp
)

string(REPLACE "_simulator" "" PROJECT_NAME_NO_SIM "${PROJECT_NAME}")

FLAIR_DEMO_HOST_CXX(${PROJECT_NAME} "${SRC_FILES}"
           DEST_DIR ${PROJECT_NAME_NO_SIM}
           COPY_RESOURCES
)