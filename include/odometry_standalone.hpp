#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <yarp/os/Bottle.h>
#include <yarp/os/Port.h>
#include <yarp/os/Network.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Time.h>

#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <chrono>
#include <thread>

using yarp::os::Bottle;

class Estimator_odom : public rclcpp::Node
{
private:
    bool initialized;
    const char* reader_port_name = "/estimator_odom/odom_reader";
    const char* writer_port = "/base-estimator/floating_base/state:o";
    const char* imu_reader_port_name = "/estimator_odom/imu_reader";
    const char* imu_writer_port = "/icubSim/chest/inertials/measures:o";
    yarp::os::Port odom_reader_port;
    yarp::os::Port imu_reader_port; //tmp
    
    //tf2_ros::TransformBroadcaster tf_broadcaster;

    geometry_msgs::msg::TransformStamped transformStamped;
    geometry_msgs::msg::TransformStamped odom_tf;
    geometry_msgs::msg::TransformStamped TF;
    nav_msgs::msg::Odometry odom_msg;
    tf2::Quaternion imu_quat;
    double imu_yaw;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::shared_ptr<tf2_ros::Buffer> tfBuffer_in;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Subscription<sensor_msgs::msg::Imu::SharedPtr>::SharedPtr chest_imu_sub;
    const char* odom_topic = "estimated_odom";
    const char* chest_imu_topic = "chest_imu";
    const char* odom_frame_name = "odom";
    const char* root_link_name = "root_link";

    bool xy_offsets_computed, yaw_offsets_computed;
    double initial_offset_x, initial_offset_y, initial_offset_yaw;
    
public:
    Estimator_odom();
    bool get_TF(const char* target_link);
    bool init();
    bool compute_odom();
    bool publish_odom();
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr &msg);
};