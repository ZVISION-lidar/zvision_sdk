cmake_minimum_required(VERSION 3.0)

#set(ZVISION_SDK_LIBRARY ${PROJECT_NAME}_static)
add_library(${SDK_LIBRARY_NAME} STATIC "")
add_compile_options(-Wall -g)

target_include_directories(${SDK_LIBRARY_NAME}
        PUBLIC
        include/common
        include
        PRIVATE
        include/commu
        include/protocol
        )

set_target_properties(${SDK_LIBRARY_NAME} PROPERTIES PUBLIC_HEADER "include/tcp_tools.h;include/point_cloud.h;include/common/define.h;include/common/print.h;include/loguru.hpp")

target_compile_options(${SDK_LIBRARY_NAME}
        #PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall -Werror -Wno-c++11-long-long>
        PRIVATE $<$<CXX_COMPILER_ID:AppleClang>:-Wno-unknown-pragmas -Wall -Werror -Wno-c++11-long-long>
        PRIVATE $<$<CXX_COMPILER_ID:Clang>:-Wno-unknown-pragmas -Wall -Werror -Wno-c++11-long-long>)

target_sources(${SDK_LIBRARY_NAME}
        PRIVATE
        src/tcp_ez6_b2.cpp
        src/tcp_nz1_a2.cpp
        src/packet_ez6_b2.cpp
        src/packet_nz1_a2.cpp
        src/packet_mrz16.cpp
        src/point_cloud.cpp
        src/print.cpp
        src/client.cpp
        src/serial_client.cpp
        src/define.cpp
        src/loguru.cpp
        )

install(TARGETS ${SDK_LIBRARY_NAME}
        ARCHIVE DESTINATION ${INSTALL_LIBRARY_DIR}
        LIBRARY DESTINATION ${INSTALL_LIBRARY_DIR}
        PUBLIC_HEADER DESTINATION ${INSTALL_INCLUDE_DIR}
        )
