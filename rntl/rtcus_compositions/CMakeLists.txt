cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(PCL 1.5 REQUIRED)
find_package( MRPT REQUIRED base) 

include_directories(${CMAKE_SOURCE_DIR}/include ${PCL_INCLUDE_DIRS})

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/poses.cpp)
rosbuild_add_library(${PROJECT_NAME} src/points.cpp)
rosbuild_add_library(${PROJECT_NAME} src/point_clouds/point_clouds_common.cpp)
rosbuild_add_library(${PROJECT_NAME} src/point_clouds/point_clouds.cpp)
rosbuild_add_library(${PROJECT_NAME} src/point_clouds/point_clouds_2d.cpp)
rosbuild_add_library(${PROJECT_NAME} src/point_clouds/point_clouds_2d.cpp) 
rosbuild_add_library(${PROJECT_NAME} src/point_clouds/point_vector_compositions.cpp)

rosbuild_add_library(${PROJECT_NAME} src/incomplete_dynamic_state_compositions.cpp)




#TESTS
rosbuild_add_gtest(test_${PROJECT_NAME} src/tests/state_composer.cpp)
target_link_libraries (test_${PROJECT_NAME} ${PROJECT_NAME} )