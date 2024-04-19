cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_INSTALL_PREFIX /usr/bin)

project(pcd2ptx)

find_package(PCL 1.10 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcd2ptx pcd2ptx.cpp)
target_link_libraries (pcd2ptx ${PCL_LIBRARIES})
install(TARGETS pcd2ptx
    DESTINATION ${CMAKE_INSTALL_PREFIX}
    )