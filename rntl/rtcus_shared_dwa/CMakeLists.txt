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

find_package(OpenCV REQUIRED)
find_package(PCL 1.5 REQUIRED)
include_directories(${CMAKE_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()
rosbuild_genmsg()

rosbuild_add_library(${PROJECT_NAME} src/heading_strategies/shared_control_heading_cost_strategy.cpp)
rosbuild_add_library(${PROJECT_NAME} src/heading_strategies/shared_control_acceleration_heading_cost_strategy.cpp)
rosbuild_add_library(${PROJECT_NAME} src/heading_strategies/shared_control_kurvature_heading_cost_strategy.cpp)
rosbuild_add_library(${PROJECT_NAME} src/shared_dwa_node_factory.cpp)

target_link_libraries(${PROJECT_NAME} rtcus_dwa rtcus_motion_models rtcus_navigation rtcus_navigation_tools ${OpenCV_LIBS})
rosbuild_link_boost(${PROJECT_NAME} signals thread)
rosbuild_add_openmp_flags(${PROJECT_NAME})

#SHARED CONTROL DWA
rosbuild_add_executable(shared_control_dwa_move_base src/shared_control_dwa_move_base.cpp)
target_link_libraries(shared_control_dwa_move_base rtcus_dwa rtcus_navigation rtcus_navigation_tools ${PROJECT_NAME})

#SIMULATED USER JOYSTICK
rosbuild_add_executable(simulate_user_joystick src/simulate_user_joystick.cpp)
target_link_libraries(simulate_user_joystick rtcus_dwa rtcus_navigation rtcus_navigation_tools ${PROJECT_NAME})

#DIRECT JOY TELEOPERATION
rosbuild_add_executable(joy_node src/joy_node.cpp)
rosbuild_add_executable(teleop_joy_proportional_to_twist src/teleop_joy_proportional_to_twist.cpp)

rosbuild_add_openmp_flags(shared_control_dwa_move_base)
rosbuild_add_openmp_flags(simulate_user_joystick)

#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
