cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#set(ROS_LINK_FLAGS "-static")

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

#target_link_libraries(example ${PROJECT_NAME})
#rosbuild_add_executable(example examples/soar_test2.cpp)

#link_directories(${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_executable(soar_test_gazebo src/soar_test_gazebo.cpp)
rosbuild_add_executable(soar_test_erratic_hokuyo src/soar_test_erratic_hokuyo.cpp)
target_link_libraries(soar_test_erratic_hokuyo ClientSML ConnectionSML SoarKernel ElementXML)
target_link_libraries(soar_test_gazebo ClientSML ConnectionSML SoarKernel ElementXML)

#rosbuild_add_rostest(test/simple_soar_test.xml)
