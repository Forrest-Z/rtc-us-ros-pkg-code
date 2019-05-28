cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Debug)
#set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_boost_directories()

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

rosbuild_genmsg()
rosbuild_gensrv()

find_package(OpenCV REQUIRED)
find_package(PCL 1.5 REQUIRED)
include_directories(${CMAKE_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

                                  
#DWA ROS STUFF
rosbuild_add_library(${PROJECT_NAME} src/simple_dwa_ros.cpp)
rosbuild_add_library(${PROJECT_NAME} src/dwa_motion_model.cpp)
rosbuild_add_library(${PROJECT_NAME} src/dwa_config.cpp)
rosbuild_add_library(${PROJECT_NAME} src/dwa_command_cost.cpp)
rosbuild_add_library(${PROJECT_NAME} src/geometrical_distances.cpp)
rosbuild_add_library(${PROJECT_NAME} src/action_space.cpp)
rosbuild_add_library(${PROJECT_NAME} src/simple_dwa.cpp)
rosbuild_add_library(${PROJECT_NAME} src/create_default_dwa_node.cpp)
rosbuild_add_library(${PROJECT_NAME} src/clearance_strategies/clearance_cost_strategy_base.cpp)

#rosbuild_add_library(${PROJECT_NAME} src/clearance_strategies/obstacle_collision_info.cpp)
#rosbuild_add_library(${PROJECT_NAME} src/clearance_strategies/separation_obstacle_cost_strategy.cpp)
#rosbuild_add_library(${PROJECT_NAME} src/clearance_strategies/default_clearance_cost_strategy.cpp)
#rosbuild_add_library(${PROJECT_NAME} src/clearance_strategies/danger_area_clearance_cost_strategy.cpp)
#rosbuild_add_library(${PROJECT_NAME} src/clearance_strategies/mixed_clearance_cost_strategy.cpp)
#rosbuild_add_library(${PROJECT_NAME} src/clearance_strategies/danger_area_clearance_cost_strategy_base.cpp)
#rosbuild_add_library(${PROJECT_NAME} src/clearance_strategies/separation_obstacle_ponderated_clearance_cost_strategy.cpp)
rosbuild_add_library(${PROJECT_NAME} src/heading_strategies/heading_cost_strategies.cpp)
rosbuild_add_library(${PROJECT_NAME} src/heading_strategies/trajectory_goal_intersection_heading_strategy.cpp)
rosbuild_add_library(${PROJECT_NAME} src/visual_representation/dynamic_window_action_space_visual_represenation.cpp)
rosbuild_add_library(${PROJECT_NAME} src/visual_representation/circular_trajectory_set_visual_representation.cpp)

target_link_libraries(${PROJECT_NAME} rtcus_motion_models rtcus_navigation rtcus_navigation_tools rtcus_conversions)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
rosbuild_link_boost(${PROJECT_NAME} signals thread)
rosbuild_add_openmp_flags(${PROJECT_NAME})

#MOVE BASE DWA
#rosbuild_add_executable(move_base_dwa src/move_base_nodes/move_base_dwa.cpp)
#target_link_libraries(move_base_dwa rtcus_navigation rtcus_navigation_tools ${PROJECT_NAME})

#TESTS
rosbuild_add_gtest(test_${PROJECT_NAME} src/tests/basic_tests.cpp)
rosbuild_add_gtest(others_tests src/tests/basic_tf_test.cpp)
rosbuild_add_gtest(others_tests src/tests/multiple_configure_servers.cpp)
target_link_libraries (test_${PROJECT_NAME} ${PROJECT_NAME} rtcus_navigation rtcus_navigation_tools rtcus_conversions)

rosbuild_add_executable(dwa_demo_test src/tests/dwa_demo_test.cpp)
target_link_libraries (dwa_demo_test rtcus_navigation rtcus_navigation_tools ${PROJECT_NAME} gtest)
rosbuild_add_rostest(tests/dwa_demo_test.test)
rosbuild_add_openmp_flags(dwa_demo_test)
