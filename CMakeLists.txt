cmake_minimum_required(VERSION 3.17)
project(LidarFactory)

set(CMAKE_CXX_STANDARD 14)
find_package(PCL REQUIRED PATHS "/home/ubuntu/Downloads/software/pcl/install")
#find_package(PCL REQUIRED)
#include_directories( "/home/ubuntu/Downloads/software/pcl/install/include/pcl-1.11/" )
#link_directories("/home/ubuntu/Downloads/software/pcl/install/lib")
#add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)

# embedding rs_driver
#list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/rs_driver/cmake)
#include(rs_driver)
#include_directories(${rs_driver_INCLUDE_DIRS})
##

add_subdirectory(${PROJECT_SOURCE_DIR}/rs_driver)
find_package(rs_driver REQUIRED)
include_directories(${rs_driver_INCLUDE_DIRS})


## embedding STSservo
file(GLOB hdrs STSservo/*.h)
file(GLOB srs STSservo/*.cpp)
#message(${hdrs})
add_library(STSservo ${hdrs} ${srs})
include_directories(./STSservo)

#file(GLOB libpcl /home/ubuntu/Downloads/software/pcl/install/lib/libpcl*)
#message(${PCL_LIBRARIES})
#list(APPEND PCL_LIBRARIES ${libpcl})
#message(${PCL_LIBRARIES})
#message(${libpcl})

find_package(OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

find_package(yaml-cpp REQUIRED)

add_executable(visualize src/main1.cpp src/manager.h src/manager.cpp)
target_link_libraries(visualize ${rs_driver_LIBRARIES} ${PCL_LIBRARIES} ${YAML_CPP_LIBRARIES} STSservo -lpthread)

add_executable(four src/four.cpp)
target_link_libraries(four ${rs_driver_LIBRARIES} ${PCL_LIBRARIES} STSservo -lpthread)

add_executable(calib src/calibrate.cpp src/ProcessPointClouds.cpp src/ProcessPointClouds.h)
target_link_libraries(calib ${PCL_LIBRARIES})

add_executable(test src/test.cpp src/ProcessPointClouds.cpp src/ProcessPointClouds.h)
target_link_libraries(test ${PCL_LIBRARIES} -lpthread)

add_executable(testSTS src/testSTS.cpp)
target_link_libraries(testSTS STSservo -lpthread)

add_executable(crush src/crush.cpp)
target_link_libraries(crush ${OpenCV_LIBRARIES})


add_executable(crush3D src/crush3D.cpp src/ProcessPointClouds.cpp src/ProcessPointClouds.h)
target_link_libraries(crush3D ${PCL_LIBRARIES})



add_executable(tsdf src/testTSDF.cpp src/sdf/sdf.cpp src/sdf/marching_cubes_sdf.cpp)
target_link_libraries(tsdf ${PCL_LIBRARIES})


