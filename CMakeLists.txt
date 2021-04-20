cmake_minimum_required(VERSION 3.17)
project(LidarFactory)

set(CMAKE_CXX_STANDARD 14)
find_package(PCL REQUIRED)
message(STATUS "PCL_DIR = ${PCL_DIR}")
include_directories (${PCL_INCLUDE_DIRS})
#link_directories    (${PCL_LIBRARY_DIRS})
#add_definitions     (${PCL_DEFINITIONS})
#find_package(PCL REQUIRED)
#include_directories( "/home/ubuntu/Downloads/software/pcl/install/include/pcl-1.11/" )
#link_directories("/home/ubuntu/Downloads/software/pcl/install/lib")
#add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV_DIR = ${OpenCV_DIR}")

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


find_package(OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

find_package(yaml-cpp REQUIRED)
include_directories (${YAML_CPP_INCLUDE_DIRS})
message(STATUS "yaml_DIR = ${yaml-cpp_DIR}")

find_package(Pistache REQUIRED PATHS "/home/ubuntu/pistache/prefix")
include_directories("/home/ubuntu/pistache/prefix/include")
include_directories("/home/ubuntu/pistache/third-party/rapidjson/include")


file(GLOB_RECURSE ALL_SRC "src/*.cpp")
file(GLOB_RECURSE APP_SRC "src/apps/*cpp")
list(REMOVE_ITEM ALL_SRC ${APP_SRC})


set(ALL_TARGET_LIBRARIES ${rs_driver_LIBRARIES} ${PCL_LIBRARIES} ${YAML_CPP_LIBRARIES} ${OpenCV_LIBRARIES} ${Pistache_LIBRARIES} STSservo -lpthread )
message(STATUS "*****= ${ALL_TARGET_LIBRARIES}")

include_directories(${PROJECT_SOURCE_DIR}/src)


add_executable(map src/apps/main1.cpp ${ALL_SRC})
target_link_libraries(map ${rs_driver_LIBRARIES} ${PCL_LIBRARIES} ${YAML_CPP_LIBRARIES} ${OpenCV_LIBRARIES} STSservo -lpthread)

add_executable(four src/apps/four.cpp ${ALL_SRC})
target_link_libraries(four ${rs_driver_LIBRARIES} ${PCL_LIBRARIES} STSservo -lpthread)

add_executable(calib src/apps/calibrate.cpp ${ALL_SRC})
target_link_libraries(calib ${PCL_LIBRARIES})

add_executable(test src/apps/test.cpp ${ALL_SRC})
target_link_libraries(test ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} -lpthread)

add_executable(test1 src/apps/test1.cpp ${ALL_SRC})
target_link_libraries(test1 ${ALL_TARGET_LIBRARIES} STSservo -lpthread)


add_executable(crush3D src/apps/crush3D.cpp src/ProcessPointClouds.cpp)
target_link_libraries(crush3D ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})



add_executable(tsdf src/apps/testTSDF.cpp src/sdf/sdf.cpp src/sdf/marching_cubes_sdf.cpp)
target_link_libraries(tsdf ${PCL_LIBRARIES})


