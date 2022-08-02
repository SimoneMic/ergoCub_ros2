#include <odometry_callback.hpp>

using std::placeholders::_1;

using namespace std::chrono_literals;

// Default constructor: creates ports and connects them
OdomPublisher::OdomPublisher()
: Node("odom_ros_interface")
{
    //ROS components init
    m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(m_odom_topic, 1000);
    m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    m_tf_buffer_in = std::make_unique<tf2_ros::Buffer>(get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_in);
}

//get the transform from frame source_link to frame target_link. 
bool OdomPublisher::get_TF(const std::string &target_link, const std::string &source_link, geometry_msgs::msg::TransformStamped &out)
{
    try
    {
        out = m_tf_buffer_in->lookupTransform(target_link, source_link, rclcpp::Time(0), 100ms);
        //RCLCPP_INFO(this->get_logger(), "TF: x %f y %f z %f  - xa %f ya %f za %f wa %f \n", TF.transform.translation.x, TF.transform.translation.y, TF.transform.translation.z,
        //                                                                                    TF.transform.rotation.x, TF.transform.rotation.y, TF.transform.rotation.z, TF.transform.rotation.w);
        return true;
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "Error in TF computation: %s \n", ex.what());
        return false;
    }
    return true;
}

bool OdomPublisher::publish_odom(geometry_msgs::msg::TransformStamped &tf, nav_msgs::msg::Odometry &odom_msg)
{
    try
    {
        std::lock_guard<std::mutex> guard(m_mutex);   //lock
        //Setting timestamps on publishing
        odom_msg.header.stamp = now();
        tf.header.stamp = odom_msg.header.stamp;
        //std::cout << "sending TF " << std::endl;
        m_tf_broadcaster->sendTransform(tf);
        //std::cout << "sending odom msg " << std::endl;
        m_odom_pub->publish(odom_msg);
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "%s \n",e.what());
        return false;
    }
    return true;
}
