cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Debug)
#set(ROS_BUILD_TYPE Debug)
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#------- REQUIRED PACKAGES -------------
find_package(PCL 1.5 REQUIRED)
find_package(CGAL 4.1 REQUIRED)
find_package( MRPT REQUIRED base maps)

include_directories(${CMAKE_SOURCE_DIR}/include ${PCL_INCLUDE_DIRS} ${CGAL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${CGALL_LIBRARY_DIRS})

rosbuild_genmsg()
rosbuild_add_boost_directories()

#----- ROS DYNAMIC RECONFIGURE---------
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()
#---------- RTCUS_NAVIGATION SOURCES ------------
rosbuild_add_library(${PROJECT_NAME} src/navigation_node_component.cpp)
rosbuild_add_library(${PROJECT_NAME} src/abstract_navigation_node.cpp)
rosbuild_add_library(${PROJECT_NAME} src/navigation_node.cpp)

rosbuild_add_library(${PROJECT_NAME} src/state_estimation.cpp)
rosbuild_add_library(${PROJECT_NAME} src/state_estimation/nonholonomic_trajectory_rollout2d.cpp)

rosbuild_add_library(${PROJECT_NAME} src/world_perception_ports/costmap_perception_port.cpp)
rosbuild_add_library(${PROJECT_NAME} src/world_perception.cpp)

rosbuild_add_library(${PROJECT_NAME} src/collision_chekers/collision_checker_pointcloud2D_circular_robot.cpp)
rosbuild_add_library(${PROJECT_NAME} src/collision_chekers/collision_checker_pointxy_circular_robot.cpp)
rosbuild_add_library(${PROJECT_NAME} src/collision_chekers/collision_cheker_polygonal_robot.cpp)
rosbuild_add_library(${PROJECT_NAME} src/collision_chekers/collision_checker_triangles.cpp)

rosbuild_add_library(${PROJECT_NAME} src/trajectory_clearance/circular_trajectory_clearance_circular_robot.cpp)
rosbuild_add_library(${PROJECT_NAME} src/trajectory_clearance/lookup_table_circular_robots.cpp)
rosbuild_add_library(${PROJECT_NAME} src/trajectory_clearance/discrete_collision_shaped_robot_clearance.cpp)
rosbuild_add_library(${PROJECT_NAME} src/trajectory_clearance/trajectory_rollout_polygonal_robot.cpp)
rosbuild_add_library(${PROJECT_NAME} src/trajectory_clearance/trajectory_rollout_circular_robot.cpp)
rosbuild_add_library(${PROJECT_NAME} src/trajectory_clearance/linear_trajectory_clearance.cpp)

#rosbuild_add_library(${PROJECT_NAME} src/trajectory_clearance/circular_trajectory_polygonal_shape_clearance.cpp)
rosbuild_add_library(${PROJECT_NAME} src/trajectory_clearance/workspace_trajectory_2d_circular_robot_clearance.cpp)

rosbuild_add_library(${PROJECT_NAME} src/action_port.cpp)
rosbuild_add_library(${PROJECT_NAME} src/action_ports/action_port_stamped_twist2d.cpp)
rosbuild_add_library(${PROJECT_NAME} src/action_ports/action_buffering.cpp)
rosbuild_add_library(${PROJECT_NAME} src/action_ports/simulate_user_joystick_action_port.cpp)

rosbuild_add_library(${PROJECT_NAME} src/goal_port.cpp)
rosbuild_add_library(${PROJECT_NAME} src/goal_ports/shared_control_proportional_dynamics.cpp)
rosbuild_add_library(${PROJECT_NAME} src/goal_ports/proportional_joy_shared_control_goal_port.cpp)
rosbuild_add_library(${PROJECT_NAME} src/goal_ports/waypoints_goal_port.cpp)

rosbuild_add_library(${PROJECT_NAME} src/state_port.cpp)
rosbuild_add_library(${PROJECT_NAME} src/state_ports/high_noise_odometry_state_port.cpp)

rosbuild_add_library(${PROJECT_NAME} src/shape_models/static_circular_reconfigurable.cpp)
rosbuild_add_library(${PROJECT_NAME} src/shape_models/static_poligonal_reconfigurable.cpp)
rosbuild_add_library(${PROJECT_NAME} src/shape_models/polygonal_reconfigurable_robot_shape_model.cpp)

rosbuild_add_library(${PROJECT_NAME} src/kinodynamic_models/non_holonomic_kinodynamic.cpp)

rosbuild_add_library(${PROJECT_NAME} src/reached_goal_predicate.cpp)
rosbuild_add_library(${PROJECT_NAME} src/goal_reach_detection/default_goal_reach_predicate.cpp)

#--- LINKING LIBRARIES
rosbuild_link_boost(${PROJECT_NAME} signals thread)
target_link_libraries (${PROJECT_NAME} rtcus_stamp rtcus_compositions rtcus_robot_shapes)
target_link_libraries (${PROJECT_NAME} ${PCL_COMMON_LIBRARIES} ${MRPT_LIBS} CGAL_Core )

#----------------- PROJECT TESTS --------------------------------------- 
rosbuild_add_gtest(${PROJECT_NAME}_test src/test/test_main.cpp)
rosbuild_add_gtest(${PROJECT_NAME}_test src/test/world_perception_tests.cpp)
rosbuild_add_gtest(${PROJECT_NAME}_test src/test/action_port_tests.cpp)
rosbuild_add_gtest(${PROJECT_NAME}_test src/test/goal_port_tests.cpp)
rosbuild_add_gtest(${PROJECT_NAME}_test src/test/state_port_tests.cpp)
rosbuild_add_gtest(${PROJECT_NAME}_test src/test/navigation_node_tests.cpp)
rosbuild_add_gtest(${PROJECT_NAME}_test src/test/lookup_table_test.cpp)
rosbuild_add_gtest(${PROJECT_NAME}_test src/test/collision_checker_tests.cpp)
rosbuild_add_gtest(${PROJECT_NAME}_test src/test/polygonal_trajectory_clearance.cpp)
rosbuild_add_gtest(${PROJECT_NAME}_test src/test/circular_shape_trajectory_rollout_clearance.cpp)
rosbuild_add_gtest(${PROJECT_NAME}_test src/test/linear_trajectory_clearance.cpp)

#rosbuild_add_gtest(${PROJECT_NAME}_quick_test src/test/test_main.cpp)


target_link_libraries (${PROJECT_NAME}_test ${PROJECT_NAME} ${PCL_COMMON_LIBRARIES} rtcus_stamp rtcus_compositions rtcus_robot_shapes rtcus_motion_models ${MRPT_LIBS})
#target_link_libraries (${PROJECT_NAME}_quick_test ${PROJECT_NAME} ${PCL_COMMON_LIBRARIES} rtcus_stamp rtcus_compositions rtcus_robot_shapes rtcus_motion_models  ${MRPT_LIBS})
rosbuild_link_boost(${PROJECT_NAME}_test signals thread)
#rosbuild_link_boost(${PROJECT_NAME}_quick_test signals thread)

#add_subdirectory(src/test)
