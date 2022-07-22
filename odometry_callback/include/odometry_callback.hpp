#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <yarp/os/Bottle.h>
#include <yarp/os/Port.h>
#include <yarp/os/Network.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Time.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <chrono>
#include <thread>

using yarp::os::Bottle;

class OdomPublisher : public rclcpp::Node
{
private:
    geometry_msgs::msg::TransformStamped TF;
    const std::string odom_topic = "odom";

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_in;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

public:
    OdomPublisher();
    bool get_TF(const std::string &target_link,const std::string &source_link, geometry_msgs::msg::TransformStamped &out);
    bool publish_odom(geometry_msgs::msg::TransformStamped &tf, nav_msgs::msg::Odometry &odom_msg);
};

