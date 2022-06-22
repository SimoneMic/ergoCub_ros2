#include "trajectory_controller.hpp"

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

#include <algorithm>
#include <string>
#include <memory>

//using std::hypot;
//using std::min;
//using std::max;
//using std::abs;
//using nav2_util::declare_parameter_if_not_declared;
//using nav2_util::geometry_utils::euclidean_distance;

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
    //nav2_util::declare_parameter_if_not_declared(
    //    node, plugin_name_ + ".error_threshold", rclcpp::ParameterValue(0.2)
    //);
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.1)
    );

    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.4)
    );
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.0)
    );

    double transform_tolerance_tmp;
    node -> get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_tmp);
    transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance_tmp);
    //node -> get_parameter(plugin_name_ + ".error_threshold", error_threshold_);
    node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
    node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
    
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
    //Send the path to walking-controller after cropping it?
    global_pub_->publish(path);
    //Write YARP port

    //Read computed (zig-zag) path from walking-controller
    //Read YARP port
    global_plan_ = path;
}

bool ErgoCubTrajectoryController::transformPose(
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::string frame,
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose,
    const rclcpp::Duration & transform_tolerance
) const
{
    // Implementation taken as is fron nav_2d_utils in nav2_dwb_controller

    //Check if already in the same frame
    if (in_pose.header.frame_id == frame) 
    {
        out_pose = in_pose;
        return true;
    }

    //Tries to ask for the TF in the buffer and transform the pose
    try 
    {
        tf->transform(in_pose, out_pose, frame);
        return true;
    } 
    catch (tf2::ExtrapolationException & ex) //error of extrapolation
    {
        //Look for available transform at the most recent time
        auto transform = tf->lookupTransform(
            frame,
            in_pose.header.frame_id,
            tf2::TimePointZero
        );
        if (
            (rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) > transform_tolerance)  // TF too old
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("tf_help"),
                "Transform data too old when converting from %s to %s",
                in_pose.header.frame_id.c_str(),
                frame.c_str()
            );
            RCLCPP_ERROR(
                rclcpp::get_logger("tf_help"),
                "Data time: %ds %uns, Transform time: %ds %uns",
                in_pose.header.stamp.sec,
                in_pose.header.stamp.nanosec,
                transform.header.stamp.sec,
                transform.header.stamp.nanosec
            );
            return false;
        } 
        else // TF ok
        {
            tf2::doTransform(in_pose, out_pose, transform);
            return true;
        }
    } catch (tf2::TransformException & ex) // transform not ready
    {
        RCLCPP_ERROR(
        rclcpp::get_logger("tf_help"),
        "Exception in transformPose: %s",
        ex.what()
        );
        return false;
    }
    return false;
}

nav_msgs::msg::Path ErgoCubTrajectoryController::transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose)
{
    // Original implementation taken fron nav2_dwb_controller

    if (global_plan_.poses.empty()) 
    {
        throw nav2_core::PlannerException("Received plan with zero length");
    }

    // let's get the pose of the robot in the frame of the plan
    geometry_msgs::msg::PoseStamped robot_pose;
    
    if (! this->transformPose( tf_buffer_, global_plan_.header.frame_id, pose,
        robot_pose, transform_tolerance_))
    {
      throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
    }

    // We'll discard points on the plan that are outside the local costmap
    nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
    double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) * costmap->getResolution() / 2.0;

    // First find the closest pose on the path to the robot
    auto transformation_begin =
        min_by(
            global_plan_.poses.begin(), global_plan_.poses.end(),
            [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
                return nav2_util::geometry_utils::euclidean_distance(robot_pose, ps);
        });

    // From the closest point, look for the first point that's further then dist_threshold from the
    // robot. These points are definitely outside of the costmap so we won't transform them.
    auto transformation_end = std::find_if(
        transformation_begin, end(global_plan_.poses),
        [&](const auto & global_plan_pose) {
            return euclidean_distance(robot_pose, global_plan_pose) > dist_threshold;
        });

    // Helper function for the transform below. Transforms a PoseStamped from global frame to local
    auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
        // We took a copy of the pose, let's lookup the transform at the current time
        geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
        stamped_pose.header.frame_id = global_plan_.header.frame_id;
        stamped_pose.header.stamp = pose.header.stamp;
        stamped_pose.pose = global_plan_pose.pose;
        transformPose(
            tf_buffer_, costmap_ros_->getBaseFrameID(),
            stamped_pose, transformed_pose, transform_tolerance_
            );
        return transformed_pose;
    };

    // Transform the near part of the global plan into the robot's frame of reference.
    nav_msgs::msg::Path transformed_plan;
    std::transform( 
        transformation_begin, transformation_end,
        std::back_inserter(transformed_plan.poses),
        transformGlobalPoseToLocal);
    
    transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
    transformed_plan.header.stamp = pose.header.stamp;

    // Remove the portion of the global plan that we've already passed so we don't
    // process it on the next iteration (this is called path pruning)
    global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
    global_pub_->publish(transformed_plan);

    if (transformed_plan.poses.empty()) {
        throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
    }

    return transformed_plan;
}

geometry_msgs::msg::TwistStamped computeVelocityCommands (
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * goal_checker)
{
    auto transformed_plan = transformGlobalPlan(pose);

    // Find the first pose which is at a distance greater than the specified lookahed distance
    auto goal_pose_it = std::find_if(
      transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
        return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist_;
      });

    // If the last pose is still within lookahed distance, take the last pose
    if (goal_pose_it == transformed_plan.poses.end()) {
      goal_pose_it = std::prev(transformed_plan.poses.end());
    }
    auto goal_pose = goal_pose_it->pose;

    double linear_vel, angular_vel;

    // If the goal pose is in front of the robot then compute the velocity using the pure pursuit
    // algorithm, else rotate with the max angular velocity until the goal pose is in front of the
    // robot
    if (goal_pose.position.x > 0) {
      auto curvature = 2.0 * goal_pose.position.y /
        (goal_pose.position.x * goal_pose.position.x + goal_pose.position.y * goal_pose.position.y);
      linear_vel = desired_linear_vel_;
      angular_vel = desired_linear_vel_ * curvature;
    } else {
      linear_vel = 0.0;
      angular_vel = max_angular_vel_;
    }
}   // namespace ergoCub_trajectory_controller