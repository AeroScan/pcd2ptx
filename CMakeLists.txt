cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

set(CMAKE_INSTALL_PREFIX /usr/bin)

project(pcd2ptx)

find_package(PCL 1.10 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB HDRS ${CMAKE_CURRENT_SOURCE_DIR}/include/*)

add_library(pcd2ptx pcd2ptx.cpp ${HDRS})
target_include_directories(pcd2ptx PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries (pcd2ptx ${PCL_LIBRARIES})


add_executable(ConversionTest tests/conversion_test.cpp)
target_link_libraries(ConversionTest pcd2ptx)

add_test(NAME Conversion_Test COMMAND ConversionTest)