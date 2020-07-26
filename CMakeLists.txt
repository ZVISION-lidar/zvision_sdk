cmake_minimum_required(VERSION 3.0)

project(zvision_sdk)

set(CMAKE_CXX_STANDARD 11)

message(STATUS "main project dir: " ${PROJECT_SOURCE_DIR})

if (CMAKE_CROSSCOMPILING)
	set(THREADS_PTHREAD_ARG
		"PLEASE_FILL_OUT-FAILED_TO_RUN"
		CACHE STRING "Result from TRY_RUN" FORCE)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif()

add_subdirectory(sdk)
add_subdirectory(sample/pointcloud)
add_subdirectory(sample/lidar_config)