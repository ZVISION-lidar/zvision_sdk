cmake_minimum_required(VERSION 3.0)

set(DEMO_NAME lidarconfig_sample)
add_executable(${DEMO_NAME} main.cpp)

target_link_libraries(${DEMO_NAME}
        ${PROJECT_NAME}_static
        )