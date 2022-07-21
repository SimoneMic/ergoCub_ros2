#include <odometry_callback.hpp>

using std::placeholders::_1;

using namespace std::chrono_literals;

// Default constructor: creates ports and connects them
OdomPublisher::OdomPublisher()
: Node("odom_ros_interface")
{
    //ROS components init
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 1000);
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_buffer_in = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_in);
}

//get the transform from frame source_link to frame target_link. 
bool OdomPublisher::get_TF(const std::string &target_link, const std::string &source_link, geometry_msgs::msg::TransformStamped &out)
{
    try
    {
        while (rclcpp::ok())
        {
            try
            {
                out = tf_buffer_in->lookupTransform(target_link, source_link, rclcpp::Time(0), 100ms);
                //RCLCPP_INFO(this->get_logger(), "TF: x %f y %f z %f  - xa %f ya %f za %f wa %f \n", TF.transform.translation.x, TF.transform.translation.y, TF.transform.translation.z,
                //                                                                                    TF.transform.rotation.x, TF.transform.rotation.y, TF.transform.rotation.z, TF.transform.rotation.w);
                return true;
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "Error in TF computation: %s \n", ex.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(50));  //rclcpp::Duration(0.1).sleep();
                continue;
            }
        }
    }
    catch(const std::exception& e)
    {
        RCLCPP_WARN(this->get_logger(), "%s \n",e.what());
        return false;
    }
    return true;
}

/*
bool OdomPublisher::compute_odom()
{
    try
    {
        Bottle contacts_bottle;
        contacts_reader_port.read(contacts_bottle);
        if (in_bottle.get(0).asFloat64() > sensor_threshold && in_bottle.get(1).asFloat64() < sensor_threshold)
        {
            //l_sole
            foot_link = "l_sole"; 
        }   
        else if (in_bottle.get(1).asFloat64() > sensor_threshold && in_bottle.get(0).asFloat64() < sensor_threshold)
        {
            //r_sole
            foot_link = "r_sole"; 
        }
        //Translation Component
        if (! get_TF("chest", root_link_name))  // updates the TF variable with the most recent tf
        {
            throw std::logic_error("Cannot get TF 1");
        }
        odom_tf.transform.translation.x = in_bottle.get(0).asFloat64() - initial_offset_x - TF.transform.translation.x;
        odom_tf.transform.translation.y = in_bottle.get(1).asFloat64() - initial_offset_y - TF.transform.translation.y;
        odom_tf.transform.translation.z = in_bottle.get(2).asFloat64() + TF.transform.translation.z; 
        //Orientation
        if (! get_TF(foot_link, root_link_name))  // updates the TF variable with the most recent tf
        {
            throw std::logic_error("Cannot get TF 2");
        }
        tf2::Quaternion tf_ground;    //quat of the root_link to chest frame tf
        tf2::fromMsg(TF.transform.rotation, tf_ground);
        tf2::Matrix3x3 m(tf_ground);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        tf2::Quaternion q_final;  //imu quaternion reading from sensor
        q_final.setRPY(roll, pitch, imu_bottle.get(3).asList()->get(0).asList()->get(0).asList()->get(2).asFloat64()* 0.0174533);
        

        odom_tf.transform.rotation.x = q_final.x();
        odom_tf.transform.rotation.y = q_final.y();
        odom_tf.transform.rotation.z = q_final.z();
        odom_tf.transform.rotation.w = q_final.w();

        // odom msg
        odom_msg.pose.pose.orientation = odom_tf.transform.rotation;
        odom_msg.pose.pose.position.x = odom_tf.transform.translation.x;
        odom_msg.pose.pose.position.y = odom_tf.transform.translation.y;
        odom_msg.pose.pose.position.z = odom_tf.transform.translation.z;
        odom_msg.twist.twist.linear.x = in_bottle.get(6).asFloat64();
        odom_msg.twist.twist.linear.y = in_bottle.get(7).asFloat64();
        odom_msg.twist.twist.linear.z = in_bottle.get(8).asFloat64();
        odom_msg.twist.twist.angular.x = in_bottle.get(9).asFloat64();
        odom_msg.twist.twist.angular.y = in_bottle.get(10).asFloat64();
        odom_msg.twist.twist.angular.z = in_bottle.get(11).asFloat64();
        
        // time stamps
        odom_msg.header.stamp = now(); // yarp::os::Time::now()
        odom_tf.header.stamp = odom_msg.header.stamp;
        //RCLCPP_INFO(this->get_logger(), "reading time: %f", now());
        //std::cout << "retunrning odom \n";
        return true;
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "%s \n",e.what());
        return false;
    }
}
*/
bool OdomPublisher::publish_odom(geometry_msgs::msg::TransformStamped &tf, nav_msgs::msg::Odometry &odom_msg)
{
    try
    {
        //Setting timestamps on publishing
        odom_msg.header.stamp = now();
        tf.header.stamp = odom_msg.header.stamp;
        //std::cout << "sending TF \n";
        tf_broadcaster->sendTransform(tf);
        //std::cout << "sending odom msg \n";
        odom_pub->publish(odom_msg);
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "%s \n",e.what());
        return false;
    }
    return true;
}
