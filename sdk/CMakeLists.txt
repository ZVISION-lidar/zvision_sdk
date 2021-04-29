cmake_minimum_required(VERSION 3.0)

set(ZVISION_SDK_LIBRARY ${PROJECT_NAME}_static)
add_library(${ZVISION_SDK_LIBRARY} STATIC "")

set(ZVISION_SDK_MAJOR_VERSION "0")
set(ZVISION_SDK_MINOR_VERSION "1")
set(ZVISION_SDK_PATCH_VERSION "8")
set(ZVISION_SDK_VERSION_STRING "${ZVISION_SDK_MAJOR_VERSION}.${ZVISION_SDK_MINOR_VERSION}.${ZVISION_SDK_PATCH_VERSION}")

target_include_directories(${ZVISION_SDK_LIBRARY}
        PUBLIC
        include/common
        include
        PRIVATE
        include/commu
        include/protocol)

set_target_properties(${ZVISION_SDK_LIBRARY} PROPERTIES PUBLIC_HEADER "include/lidar_tools.h;include/point_cloud.h;include/common/define.h;include/common/print.h")

target_compile_options(${ZVISION_SDK_LIBRARY}
        #PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall -Werror -Wno-c++11-long-long>
        PRIVATE $<$<CXX_COMPILER_ID:AppleClang>:-Wno-unknown-pragmas -Wall -Werror -Wno-c++11-long-long>
        PRIVATE $<$<CXX_COMPILER_ID:Clang>:-Wno-unknown-pragmas -Wall -Werror -Wno-c++11-long-long>)

target_sources(${ZVISION_SDK_LIBRARY}
        PRIVATE
        src/lidar_tools.cpp
        src/packet.cpp
        src/packet_source.cpp
        src/point_cloud.cpp
        src/print.cpp
        src/client.cpp
        src/define.cpp)

install(TARGETS ${ZVISION_SDK_LIBRARY}
        PUBLIC_HEADER DESTINATION include
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)
