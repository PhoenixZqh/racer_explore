cmake_minimum_required(VERSION 2.8.3)
project(msg_set)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11)
ADD_COMPILE_OPTIONS(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# set(CMAKE_CXX_STANDARD 14)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	geometry_msgs
	message_generation
	cv_bridge
	nav_msgs
	mavros_msgs
)

add_message_files(
	FILES
	DroneState.msg
)

generate_messages(
	DEPENDENCIES
	geometry_msgs
	nav_msgs
	std_msgs
	mavros_msgs
)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS message_runtime

	# DEPENDS system_lib
)

include_directories(
	include
	SYSTEM
	${catkin_INCLUDE_DIRS}
	${PROJECT_SOURCE_DIR}/include
	${EIGEN3_INCLUDE_DIR}

	# ${PCL_INCLUDE_DIRS}
)

# ${Python3.8_INLCUDE_DIRS}
