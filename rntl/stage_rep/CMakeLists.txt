cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

include(FindPkgConfig)
rosbuild_init()

rosbuild_make_distribution(1.6.6)
rosbuild_add_boost_directories()

# Build Stage
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.stage
                RESULT_VARIABLE _make_failed)
if(_make_failed)
  message(FATAL_ERROR "Build of Stage failed")
endif(_make_failed)



# Find the build flags for Stage
set(ENV{PKG_CONFIG_PATH} "${PROJECT_SOURCE_DIR}/lib64/pkgconfig:${PROJECT_SOURCE_DIR}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")
pkg_check_modules(STAGE REQUIRED stage)
find_package(PCL 1.5 REQUIRED)
find_package(OpenCV REQUIRED)

# Adding explicit stage include and link directories since pkgconfig may not be correct
# when building for a deb install.
include_directories(${STAGE_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include/Stage-4.1 ${PROJECT_SOURCE_DIR}/include ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} 
${PROJECT_SOURCE_DIR}/include2)
link_directories(${STAGE_LIBRARY_DIRS} ${PROJECT_SOURCE_DIR}/lib ${PROJECT_SOURCE_DIR}/lib64)



rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()
rosbuild_genmsg()

rosbuild_add_executable(bin/stageros src/stageros.cpp)
rosbuild_add_executable(bin/stageros src/dynamic_reconfigure_stageros.cpp)
rosbuild_link_boost(bin/stageros thread filesystem system)
add_dependencies(bin/stageros build_stage)

rosbuild_add_compile_flags(bin/stageros ${STAGE_CFLAGS_OTHERS})
target_link_libraries(bin/stageros ${STAGE_LIBRARIES}  ${OpenCV_LIBS} rtcus_robot_shapes)
rosbuild_add_link_flags(bin/stageros ${STAGE_LDFLAGS_OTHERS} )


rosbuild_add_rostest(test/hztest.xml)
