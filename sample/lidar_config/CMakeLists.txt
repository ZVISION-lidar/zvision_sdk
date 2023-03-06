cmake_minimum_required(VERSION 3.0)

set(DEMO_NAME lidarconfig_sample)
add_executable(${DEMO_NAME} main.cpp
               convert.hpp
              )

add_executable(${DEMO_NAME}_ml30sp_b1 sample_ml30sp_b1.cpp
               convert.hpp
              )

target_link_libraries(${DEMO_NAME}
        ${PROJECT_NAME}_static
        -lpthread
        )

target_link_libraries(${DEMO_NAME}_ml30sp_b1
        ${PROJECT_NAME}_static
        -lpthread
        )