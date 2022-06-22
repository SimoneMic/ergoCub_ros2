#ifndef NAV2_TRAJECTORY_CONTROLLER__TRAJECTORY_CONTROLLE_HPP_
#define NAV2_TRAJECTORY_CONTROLLER__TRAJECTORY_CONTROLLE_HPP_


#include "rclcpp/rclcpp.hpp"
#include "nav2_core/controller.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <string>
#include <memory>
#include <vector>

namespace ergoCub_trajectory_controller
{

class ErgoCubTrajectoryController : public nav2_core::Controller
{
protected:
    nav_msgs::msg::Path transform_globalPlan(const geometry_msgs::msg::PoseStamped & pose); //still needed to have it in local frame

    bool transformPose(
        const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::string frame,
        const geometry_msgs::msg::PoseStamped & in_pose,
        geometry_msgs::msg::PoseStamped & out_pose,
        const rclcpp::Duration & transform_tolerance
    ) const;

    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string plugin_name_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    rclcpp::Logger logger_ {rclcpp::get_logger("PurePursuitController")};
    rclcpp::Clock::SharedPtr clock_;

    rclcpp::Duration transform_tolerance_ {0, 0};
    double error_threshold_;

    nav_msgs::msg::Path global_plan_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_pub_;

    /* to remove: only used in initial version of algorithm*/
    double desired_linear_vel_;
    double lookahead_dist_;
    double max_angular_vel_;

    

public:
    ErgoCubTrajectoryController() = default;
    ~ErgoCubTrajectoryController() override = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros
    );

    void cleanup() override;
    void activate() override;
    void deactivate() override;

    //compute velocity commands - can't be skipped - must be implemented but will not be used
      
    geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * goal_checker) override;

    // Could set a new plan or tell to global planner to replan
    void setPlan(const nav_msgs::msg::Path & path) override;
    nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);
};

}


#endif  // NAV2_TRAJECTORY_CONTROLLER__TRAJECTORY_CONTROLLE_HPP_