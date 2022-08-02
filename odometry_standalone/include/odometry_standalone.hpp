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

class Estimator_odom : public rclcpp::Node
{
private:
    const char* m_reader_port_name = "/estimator_odom/odom_reader:i";
    const char* m_writer_port = "/base-estimator/floating_base/state:o";
    const char* m_imu_reader_port_name = "/estimator_odom/imu_reader:i";
    const char* m_imu_writer_port = "/icubSim/chest/inertials/measures:o";
    yarp::os::Port m_odom_reader_port;
    yarp::os::Port m_imu_reader_port;
    
    geometry_msgs::msg::TransformStamped m_transformStamped;
    geometry_msgs::msg::TransformStamped m_odom_tf;
    geometry_msgs::msg::TransformStamped m_TF;
    nav_msgs::msg::Odometry m_odom_msg;
    tf2::Quaternion m_imu_quat;
    double m_imu_yaw;

    rclcpp::TimerBase::SharedPtr m_timer_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer_in;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;
    const char* m_odom_topic = "estimated_odom";
    const char* m_odom_frame_name = "odom";
    const char* m_root_link_name = "root_link";

    bool m_xy_offsets_computed, m_yaw_offsets_computed;
    double m_initial_offset_x, m_initial_offset_y, m_initial_offset_yaw;
    const double m_loopFreq = 100.0;
    const double m_deg_to_rad = M_PI/180;
    yarp::os::Port m_contacts_reader_port;
    const std::string m_contacts_reader_port_name = "/estimator_odom/contact_reader:i";
    const std::string m_contacts_server_port_name = "/base-estimator/contacts/stateAndNormalForce:o";
    const double m_sensor_threshold = 100.0;
    char* m_foot_link;
public:
    Estimator_odom();
    bool get_TF(const char* target_link,const char* source_link);
    bool compute_odom();
    bool publish_odom();
    void timer_callback();
};