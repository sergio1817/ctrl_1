PROJECT(ctrl1_RL_t)
cmake_minimum_required(VERSION 3.5)

### Eigen
# Module only if FindEigen3 exits
#list(APPEND CMAKE_MODULE_PATH "$ENV{Eigen3_MODULE}")
#SET( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )
#INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )
#find_package (Eigen3 REQUIRED)
#link_directories(${Eigen_INCLUDE_DIRS})

# No module
# set(Eigen3_DIR "$ENV{Eigen3_DIR_ARM7a}")
# SET( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR_ARM7a}" )
# INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )
# find_package (Eigen3 REQUIRED NO_MODULE)
# link_directories(${Eigen_INCLUDE_DIRS})

# message(Eigen3_DIR="${Eigen3_DIR}")
# message(EIGEN3_INCLUDE_DIR="${EIGEN3_INCLUDE_DIR}")

set(EIGEN3_INCLUDE_DIR
    "$ENV{HOME}/lib/eigen-3.4.1"
    CACHE PATH "Eigen include dir")

include_directories(${EIGEN3_INCLUDE_DIR})
message(STATUS "Using Eigen from: ${EIGEN3_INCLUDE_DIR}")


SET(FLAIR_USE_FILTER TRUE)
SET(FLAIR_USE_SENSOR_ACTUATOR TRUE)
SET(FLAIR_USE_META TRUE)
SET(FLAIR_USE_VRPN TRUE)

#SET(delivery_root_ADDRESS_PATH bin/arm)

#SET(USE_SOURCE_PERMISSIONS TRUE)

include($ENV{FLAIR_ROOT}/flair-src/cmake-modules/GlobalCmakeFlair.cmake)
#include($ENV{FLAIR_ROOT}/flair-dev/cmake-modules/GlobalCmakeFlair.cmake)

option(CTRL1_SIZE_OPTIMIZE "Optimize for size (AR.Drone 2.0: ARM Cortex-A8, 256MB)" ON)
if(CTRL1_SIZE_OPTIMIZE)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Os -ffunction-sections -fdata-sections")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os -ffunction-sections -fdata-sections")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--gc-sections -Wl,-s")
endif()


SET(SRC_FILES
	src/main.cpp
	src/ctrl1.cpp
	src/Sliding.cpp
	src/Sliding_pos.cpp
	#src/Sliding_force.cpp
	src/NMethods.cpp
	#src/MetaJR3.cpp
	#src/TargetJR3.cpp
	#src/TargetEthJR3.cpp
	src/AC1.cpp
	src/TrajectoryManager.cpp
)

FLAIR_DEMO(${PROJECT_NAME} "${SRC_FILES}"
           DEST_DIR ${PROJECT_NAME}
           COPY_RESOURCES
)
