cmake_minimum_required(VERSION 3.14)
project(pcl_object_detection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall -pedantic -Wextra")


#include_directories(third_parties/eigen-3.3.7)
#include_directories(third_parties/catch2-2.7.0)
#include_directories(${MKLROOT}/include $(MKLROOT)/../compiler/lib/$(_IA))

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_object_detection src/main.cpp)
#target_link_libraries (lidar_obstacle_detection ${PCL_LIBRARIES} -L${MKLROOT}/lib/intel64 -lmkl_sycl -lmkl_intel_ilp64 -lmkl_sequential -lmkl_core -lsycl -lOpenCL -lpthread -lm -ldl)
target_link_libraries (pcl_object_detection ${PCL_LIBRARIES})

add_library(Catch INTERFACE)
