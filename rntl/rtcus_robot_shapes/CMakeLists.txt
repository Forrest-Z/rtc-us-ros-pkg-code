cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#------- REQUIRED PACKAGES -------------
find_package(PCL 1.5 REQUIRED)
find_package(CGAL 4.1 REQUIRED) 
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frounding-math")
  
#----- ROS DYNAMIC RECONFIGURE---------
include_directories(${CMAKE_SOURCE_DIR}/include ${PCL_INCLUDE_DIRS})
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#------- PROJECT SOURCES ----------------
rosbuild_add_library(${PROJECT_NAME} src/circular_robot_shape.cpp)
rosbuild_add_library(${PROJECT_NAME} src/polygonal_robot_shape.cpp)
rosbuild_add_library(${PROJECT_NAME} src/shape_operations.cpp)

target_link_libraries(${PROJECT_NAME} CGAL CGAL_Core gmp)



include_directories(${GTEST_INCLUDE_DIR})
rosbuild_add_executable(${PROJECT_NAME}_test src/test/shape_tests.cpp)
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})




#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
