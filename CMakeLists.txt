cmake_minimum_required(VERSION 3.8)
project(ergoCub_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)
FIND_PACKAGE(YARP COMPONENTS os sig REQUIRED)
find_package(PCL 1.2 REQUIRED)

find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_kdl REQUIRED)

find_package(iDynTree 4.3.0 REQUIRED)
find_package(UnicyclePlanner 0.5.2 REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(/opt/ros/humble/include/tf2_geometry_msgs)
include_directories(/opt/ros/humble/include/tf2)
include_directories(/opt/ros/humble/include/tf2_ros)
include_directories(/opt/ros/humble/include/laser_geometry)
include_directories(/opt/ros/humble/include/pcl_conversions)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

nav2_package()

add_executable(ProtoCode unicyclePlanner_tests/ProtoCode.cpp)
target_link_libraries(ProtoCode UnicyclePlanner ${iDynTree_LIBRARIES} 
                      )
ament_target_dependencies(ProtoCode
  "std_msgs"
  "tf2_ros"
  "tf2_msgs"
  "nav_msgs"
  "rclcpp"
  "visualization_msgs"
  "UnicyclePlanner"
  "tf2"
  "nav2_util"
)

add_executable(odometry_standalone odometry_standalone/main.cpp odometry_standalone/src/odometry_standalone.cpp)
ament_target_dependencies(odometry_standalone
  "std_msgs"
  "tf2_ros"
  "tf2_msgs"
  "YARP"
  "nav_msgs"
  "rclcpp"
)
target_include_directories(odometry_standalone PRIVATE ${CMAKE_SOURCE_DIR}/odometry_standalone/include)

add_executable(odometry_callback odometry_callback/main.cpp odometry_callback/src/odometry_callback.cpp odometry_callback/src/yarp_odometry_processor.cpp)
ament_target_dependencies(odometry_callback
  "std_msgs"
  "tf2_ros"
  "tf2_msgs"
  "YARP"
  "nav_msgs"
  "rclcpp"
)
target_include_directories(odometry_callback PRIVATE ${CMAKE_SOURCE_DIR}/odometry_callback/include)

add_executable(odom_GT src/odom_GT.cpp)
ament_target_dependencies(odom_GT
  "std_msgs"
  "tf2_ros"
  "tf2_msgs"
  "YARP"
  "nav_msgs"
  "rclcpp"
)

add_executable(scan_filter src/scan_filtering_ros2.cpp)
ament_target_dependencies(scan_filter
  "std_msgs"
  "tf2_ros"
  "tf2"
  "tf2_msgs"
  "rclcpp"
  "laser_geometry"
  "pcl_conversions"
  "sensor_msgs"
)
target_link_libraries(scan_filter ${PCL_LIBRARIES})

add_executable(scan_filter_compensated src/scan_filtering_ros2_compensated.cpp)
ament_target_dependencies(scan_filter_compensated
  "std_msgs"
  "tf2_ros"
  "tf2_msgs"
  "rclcpp"
  "laser_geometry"
  "pcl_conversions"
  "geometry_msgs"
  "sensor_msgs"
)
target_link_libraries(scan_filter_compensated ${PCL_LIBRARIES})

add_executable(scan_filter_simple src/scan_filtering_ros2_simple.cpp)
ament_target_dependencies(scan_filter_simple
  "std_msgs"
  "tf2_ros"
  "tf2"
  "tf2_msgs"
  "rclcpp"
  "laser_geometry"
  "pcl_conversions"
  "sensor_msgs"
)
target_link_libraries(scan_filter_simple ${PCL_LIBRARIES})

add_executable(chest_projection src/chest_projection.cpp)
ament_target_dependencies(chest_projection
  "std_msgs"
  "tf2_ros"
  "tf2_msgs"
  "YARP"
  "rclcpp"
)

add_executable(velocity_setpoint_converter src/velocity_setpoint_converter.cpp)
ament_target_dependencies(velocity_setpoint_converter
  "YARP"
  "rclcpp"
  "geometry_msgs"
)

add_executable(piecewise_path_converter src/piecewise_path_converter.cpp)
ament_target_dependencies(piecewise_path_converter
  "YARP"
  "rclcpp"
  "nav_msgs"
  "tf2_ros"
  "tf2_msgs"
  "tf2"
  "nav2_util"
  "nav_2d_msgs"
)

#add_executable(walking_controller_talker src/walking_controller_talker.cpp)
#ament_target_dependencies(walking_controller_talker
#  "YARP"
#  "rclcpp"
#  "nav_msgs"
#  "tf2_ros"
#  "tf2_msgs"
#  "tf2"
#  "nav2_util"
#  "nav_2d_msgs"
#)

add_executable(async_walking_controller_talker src/async_walking_controller_talker.cpp)
ament_target_dependencies(async_walking_controller_talker
  "YARP"
  "rclcpp"
  "nav_msgs"
  "tf2_ros"
  "tf2_msgs"
  "tf2"
  "nav2_util"
  "nav_2d_msgs"
)

add_executable(footsteps_viewer src/footsteps_viewer.cpp)
ament_target_dependencies(footsteps_viewer
  "YARP"
  "rclcpp"
  "nav_msgs"
  "tf2_ros"
  "tf2_msgs"
  "tf2"
  "nav2_util"
  "nav_2d_msgs"
  "visualization_msgs"
)

add_executable(virtual_unicycle_publisher src/virtual_unicycle_publisher.cpp)
ament_target_dependencies(virtual_unicycle_publisher
  "YARP"
  "rclcpp"
  "tf2_ros"
  "tf2_msgs"
  "tf2"
  "tf2_kdl"
)

add_executable(CoM_trajectory_publisher src/CoM_trajectory_publisher.cpp)
ament_target_dependencies(CoM_trajectory_publisher
  "nav_msgs"
  "YARP"
  "rclcpp"
  "geometry_msgs"
)

add_executable(obstacle_script src/obstacle_script.cpp)
ament_target_dependencies(obstacle_script
  "YARP"
)

add_executable(planner_trigger_server planner_trigger/planner_trigger_server.cpp)
ament_target_dependencies(planner_trigger_server
  "YARP"
  "rclcpp"
  "std_srvs"
)

add_executable(dummy_planner unicyclePlanner_tests/dummy_planner.cpp)
ament_target_dependencies(dummy_planner
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
)

set(dependencies
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  tf2
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  #scan_filter
  scan_filter_compensated
  #scan_filter_simple
  odometry_standalone
  odometry_callback
  odom_GT
  chest_projection
  CoM_trajectory_publisher
  obstacle_script
  #velocity_setpoint_converter
  #piecewise_path_converter
  ProtoCode
  #walking_controller_talker
  footsteps_viewer
  virtual_unicycle_publisher
  async_walking_controller_talker
  planner_trigger_server
  dummy_planner
  DESTINATION lib/${PROJECT_NAME})

# install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
# install the maps directory
install(DIRECTORY
  maps
  DESTINATION share/${PROJECT_NAME}/
)
# install the param directory
install(DIRECTORY
  param
  DESTINATION share/${PROJECT_NAME}/
)
# install the RVIZ directory
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()