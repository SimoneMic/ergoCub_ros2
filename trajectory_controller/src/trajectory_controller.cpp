#include "trajectory_controller.hpp"

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

#include <algorithm>
#include <string>
#include <memory>

namespace ergoCub_trajectory_controller
{
//Find the element in the iterator with minimum calculated value
template<typename Iter, typename Getter>
Iter min_by(Iter begin, Iter end, Getter getCompareVal)
{
    if (begin==end)
    {
        return end;
    }
    auto lowest = getCompareVal(*begin);
    Iter lowest_it = begin;
    for (Iter i = ++begin; i != end; ++i)
    {
        auto comp = getCompareVal(*i);
        if (comp < lowest)
        {
            lowest = comp;
            lowest_it = i;
        }
        return lowest_it;
    }
}

void ErgoCubTrajectoryController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros
)
{
    node_ = parent; //passing node handler

    auto node = node_.lock();   //checking weak_ptr integrity

    costmap_ros_ = costmap_ros; //updating costmap taken from navigation stack
    tf_buffer_ = tf_buffer;     //passing tf buffer
    plugin_name_ = name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    // get params from launch or yaml file
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.2)
    );
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".error_threshold", rclcpp::ParameterValue(0.2)
    );
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".tbd", rclcpp::ParameterValue(0.1)
    );

    double transform_tolerance_tmp;
    node -> get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_tmp);
    transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance_tmp);
    node -> get_parameter(plugin_name_ + ".error_threshold", error_threshold_);
    
    global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
}

void ErgoCubTrajectoryController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type pure_pursuit_controller::ErgoCubTrajectoryController",
    plugin_name_.c_str());
  global_pub_.reset();
}

void ErgoCubTrajectoryController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type pure_pursuit_controller::ErgoCubTrajectoryController\"  %s",
    plugin_name_.c_str());
  global_pub_->on_activate();
}

void ErgoCubTrajectoryController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Dectivating controller: %s of type pure_pursuit_controller::ErgoCubTrajectoryController\"  %s",
    plugin_name_.c_str());
  global_pub_->on_deactivate();
}

void ErgoCubTrajectoryController::setPlan(const nav_msgs::msg::Path & path)
{
  global_pub_->publish(path);
  global_plan_ = path;
}

}   // namespace ergoCub_trajectory_controller