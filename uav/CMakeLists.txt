PROJECT(ctrl1)
cmake_minimum_required(VERSION 2.8)

SET(FLAIR_USE_FILTER TRUE)
SET(FLAIR_USE_SENSOR_ACTUATOR TRUE)
SET(FLAIR_USE_META TRUE)
SET(FLAIR_USE_VRPN TRUE)

#SET(delivery_root_ADDRESS_PATH bin/arm)

#SET(USE_SOURCE_PERMISSIONS TRUE)

include($ENV{FLAIR_ROOT}/flair-src/cmake-modules/GlobalCmakeFlair.cmake)
include($ENV{FLAIR_ROOT}/flair-dev/cmake-modules/GlobalCmakeFlair.cmake)


SET(SRC_FILES
	src/main.cpp
	src/ctrl1.cpp
	src/Sliding.cpp
	src/Sliding_pos.cpp
	src/Rk4.cpp
	#src/MetaJR3.cpp
	src/TargetJR3.cpp
	src/TargetEthJR3.cpp
)

FLAIR_DEMO(${PROJECT_NAME} "${SRC_FILES}"
           DEST_DIR ctrl1
           COPY_RESOURCES
)

