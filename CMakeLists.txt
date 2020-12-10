cmake_minimum_required(VERSION 3.17)
project(LidarFactory)


#find_package(PCL REQUIRED PATHS "/home/ubuntu/Downloads/software/pcl/build1")
find_package(PCL REQUIRED)
#include_directories( "/usr/local/include/pcl-1.11" )
#link_directories("/usr/lib/x86_64-linux-gnu/libpcl_common.so" "/usr/lib/x86_64-linux-gnu/libpcl_io.so")

find_package(OpenCV REQUIRED)

# embedding rs_driver
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/rs_driver/cmake)
include(rs_driver)
include_directories(${rs_driver_INCLUDE_DIRS})
##

## embedding STSservo
file(GLOB hdrs STSservo/*.h)
file(GLOB srs STSservo/*.cpp)
#message(${hdrs})
add_library(STSservo ${hdrs} ${srs})
include_directories(./STSservo)







add_executable(visualize src/main1.cpp)
target_link_libraries(visualize ${rs_driver_LIBRARIES} ${PCL_LIBRARIES} STSservo)

add_executable(calib src/calibrate.cpp src/ProcessPointClouds.cpp src/ProcessPointClouds.h)
target_link_libraries(calib ${PCL_LIBRARIES})

add_executable(test src/test.cpp src/ProcessPointClouds.cpp src/ProcessPointClouds.h)
target_link_libraries(test ${PCL_LIBRARIES})

add_executable(testSTS src/testSTS.cpp)
target_link_libraries(testSTS STSservo)

add_executable(crush src/crush.cpp)
target_link_libraries(crush ${OpenCV_LIBRARIES})


add_executable(crush3D src/crush3D.cpp src/ProcessPointClouds.cpp src/ProcessPointClouds.h)
target_link_libraries(crush3D ${PCL_LIBRARIES})

add_executable(edge edge.cpp)
target_link_libraries(edge ${PCL_LIBRARIES})


