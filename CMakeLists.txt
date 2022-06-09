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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)
FIND_PACKAGE(YARP COMPONENTS os REQUIRED)
find_package(PCL 1.2 REQUIRED)

include_directories(/opt/ros/humble/include/tf2_geometry_msgs)
include_directories(/opt/ros/humble/include/tf2)
include_directories(/opt/ros/humble/include/tf2_ros)
include_directories(/opt/ros/humble/include/laser_geometry)
include_directories(/opt/ros/humble/include/pcl_conversions)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(managed_odom src/odometry_lifeCycle.cpp)
ament_target_dependencies(managed_odom
  "lifecycle_msgs"
  "rclcpp_lifecycle"
  "std_msgs"
  "tf2_ros"
  "tf2_msgs"
  "YARP"
  "nav_msgs"
  "rclcpp"
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
  managed_odom
  scan_filter
  odometry_standalone
  DESTINATION lib/${PROJECT_NAME})

ament_package()
