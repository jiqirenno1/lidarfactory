cmake_minimum_required(VERSION 3.17)
project(LidarFactory)

find_package(PCL REQUIRED)

# embedding rs_driver
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/rs_driver/cmake)
include(rs_driver)
include_directories(${rs_driver_INCLUDE_DIRS})
##

add_executable(visualize src/main1.cpp)
target_link_libraries(visualize ${rs_driver_LIBRARIES} ${PCL_LIBRARIES})