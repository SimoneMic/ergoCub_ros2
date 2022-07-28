#include <yarp/os/Bottle.h>
#include <yarp/os/Port.h>
#include <yarp/os/Network.h>
#include <yarp/os/RpcClient.h>

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>

class odom_GT : public rclcpp::Node
{
private:
    //YARP
    const std::string world_port_name = "/world_input_port";
    const std::string client_port_name = "/odom_GT/client";
    yarp::os::RpcClient client_port;
    //ROS
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_gt_pub;
    const std::string topic_name = "odom_gt";
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();
    const double loopFreq = 100.0;
    nav_msgs::msg::Odometry odom_msg;
    //TF
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    geometry_msgs::msg::TransformStamped odom_gt_tf;

public:
    odom_GT(/* args */) : rclcpp::Node("odom_GT_node")
    {
        /* init YARP*/
        client_port.open(client_port_name);
        yarp::os::Network::connect(client_port_name, world_port_name);
        /* init ROS2*/
        auto duration = std::chrono::duration<double>(1/loopFreq);
        timer_ = this->create_wall_timer( duration , std::bind(&odom_GT::timer_callback, this));
        odom_gt_pub = this->create_publisher<nav_msgs::msg::Odometry>(topic_name, 10);
        /* init TFs*/
        tf_pub = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        odom_gt_tf.header.frame_id = "odom";
        odom_gt_tf.child_frame_id = "root_link";
        odom_msg.header.frame_id = odom_gt_tf.header.frame_id;
    };
};

void odom_GT::timer_callback()
{
    //Get data from rpc port
    yarp::os::Bottle cmd, response;
    cmd.addString("getPose");
    cmd.addString("stickBot");
    client_port.write(cmd, response);
    odom_gt_tf.header.stamp = now();
    odom_gt_tf.transform.translation.x = response.get(0).asFloat64();
    odom_gt_tf.transform.translation.y = response.get(1).asFloat64();
    odom_gt_tf.transform.translation.z = response.get(2).asFloat64();
    tf2::Quaternion q;
    q.setRPY(response.get(3).asFloat64(),
             response.get(4).asFloat64(),
             response.get(5).asFloat64());
    odom_gt_tf.transform.rotation.x = q.x();
    odom_gt_tf.transform.rotation.y = q.y();
    odom_gt_tf.transform.rotation.z = q.z();
    odom_gt_tf.transform.rotation.w = q.w();
    tf_pub->sendTransform(odom_gt_tf);
}

int main(int argc, char** argv)
{
    yarp::os::Network yarp;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<odom_GT>();
    if (rclcpp::ok()) 
    {
        std::cout << "Spinning odom_GT node \n";
        rclcpp::spin(node);
    }
    std::cout << "Shutting down" << std::endl;
    rclcpp::shutdown();
    return 0;
}
