cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Debug)
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()
#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/motion_models.cpp)
rosbuild_add_library(${PROJECT_NAME} src/non_holonomic_trajectory_rollout_2d.cpp)
target_link_libraries(${PROJECT_NAME} rtcus_compositions)

rosbuild_add_gtest(${PROJECT_NAME}_test src/test/motion_model_tests.cpp)
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} rtcus_compositions)

find_package(PythonLibs REQUIRED)

rosbuild_add_boost_directories()
message("Include dirs of Python: " ${PYTHON_INCLUDE_DIRS} )
message("Libs of Python: " ${PYTHON_LIBRARIES} )
include_directories(  ${PYTHON_INCLUDE_DIRS})
      

#rosbuild_add_library(python_${PROJECT_NAME} src/python_binding.cpp)
#rosbuild_link_boost(python_${PROJECT_NAME} python)

