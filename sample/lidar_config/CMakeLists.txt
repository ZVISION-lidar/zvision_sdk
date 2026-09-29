cmake_minimum_required(VERSION 3.0)

set(DEMO_NAME lidar_config)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(${DEMO_NAME} lidar_config_demo.cpp
               convert.hpp
        )
		
target_link_libraries(${DEMO_NAME}
        ${SDK_LIBRARY_NAME}
        -lpthread
        )