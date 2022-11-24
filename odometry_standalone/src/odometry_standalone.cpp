#include <odometry_standalone.hpp>

using std::placeholders::_1;

using namespace std::chrono_literals;

// Default constructor: creates ports and connects them
Estimator_odom::Estimator_odom()
: Node("odometry_node")
{
    m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(m_odom_topic, 10);

    // YARP Port Connection
    m_odom_reader_port.open(m_reader_port_name);
    m_imu_reader_port.open(m_imu_reader_port_name);
    m_contacts_reader_port.open(m_contacts_reader_port_name);
    printf("Trying to connect to %s\n", m_writer_port);
    yarp::os::Network::connect(m_writer_port, m_reader_port_name);
    yarp::os::Network::connect(m_imu_writer_port, m_imu_reader_port_name);
    yarp::os::Network::connect(m_contacts_server_port_name, m_contacts_reader_port_name);
    m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    m_tf_buffer_in = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_in);

    m_xy_offsets_computed = false;
    m_yaw_offsets_computed = false;
    m_initial_offset_x = 0;
    m_initial_offset_y = 0;
    m_initial_offset_yaw = 0;
    m_imu_yaw = 0;
    /* set up odom msg */
    m_odom_msg.header.frame_id = m_odom_frame_name;
    m_odom_msg.child_frame_id = m_root_link_name;
    /* set up odom frame */
    m_odom_tf.child_frame_id = m_root_link_name;
    m_odom_tf.header.frame_id = m_odom_frame_name;
    // TODO - Tune covariance matricies
    m_odom_msg.pose.covariance = {0.05, 0    , 0   , 0   , 0   , 0   , 
                                0   , 0.05 , 0   , 0   , 0   , 0   , 
                                0   , 0    , 0.02, 0   , 0   , 0   , 
                                0   , 0    , 0   , 0.06, 0   , 0   ,
                                0   , 0    , 0   , 0   , 0.06, 0   , 
                                0   , 0    , 0   , 0   , 0   , 0.06};
    m_odom_msg.twist.covariance = {0.1, 0    , 0   , 0   , 0   , 0   , 
                                0   , 0.1  , 0   , 0   , 0   , 0   , 
                                0   , 0    , 0.05, 0   , 0   , 0   , 
                                0   , 0    , 0   , 0.12, 0   , 0   ,
                                0   , 0    , 0   , 0   , 0.12, 0   , 
                                0   , 0    , 0   , 0   , 0   , 0.12};
    //std::cout << "creating wall timer \n";
    auto duration = std::chrono::duration<double>(1/m_loopFreq);
    m_timer_ = this->create_wall_timer( duration, std::bind(&Estimator_odom::timer_callback, this));

    m_foot_link = "r_sole"; 
}

//get the transform from frame source_link to frame target_link. 
bool Estimator_odom::get_TF(const char* target_link, const char* source_link)
{
    try
    {
        //std::cout << "looking for transform \n";
        //TF = tf_buffer_in->lookupTransform(root_link_name, target_link, rclcpp::Time(0), 100ms);
        m_TF = m_tf_buffer_in->lookupTransform(target_link, source_link, m_odom_tf.header.stamp, 50ms);
        //RCLCPP_INFO(this->get_logger(), "TF: x %f y %f z %f  - xa %f ya %f za %f wa %f \n", TF.transform.translation.x, TF.transform.translation.y, TF.transform.translation.z,
        //                                                                                    TF.transform.rotation.x, TF.transform.rotation.y, TF.transform.rotation.z, TF.transform.rotation.w);
        return true;
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "Error in TF odom computation: %s \n", ex.what());
        return false;
    }
    return true;
}

bool Estimator_odom::compute_odom()
{
    try
    {
        Bottle in_bottle;
        m_odom_reader_port.read(in_bottle);
        Bottle imu_bottle;
        m_imu_reader_port.read(imu_bottle);
        if (! m_xy_offsets_computed)
        {
            m_initial_offset_x = in_bottle.get(0).asFloat64();
            m_initial_offset_y = in_bottle.get(1).asFloat64();
            m_initial_offset_yaw = in_bottle.get(5).asFloat64();
            m_xy_offsets_computed = true;
        }
        // time stamps
        m_odom_msg.header.stamp = now();
        m_odom_tf.header.stamp = m_odom_msg.header.stamp;

        Bottle contacts_bottle;
        m_contacts_reader_port.read(contacts_bottle);
        if (contacts_bottle.get(0).asFloat64() > m_sensor_threshold && contacts_bottle.get(1).asFloat64() < m_sensor_threshold)
        {
            //l_sole
            m_foot_link = "l_sole"; 
        }   
        else if (contacts_bottle.get(1).asFloat64() > m_sensor_threshold && contacts_bottle.get(0).asFloat64() < m_sensor_threshold)
        {
            //r_sole
            m_foot_link = "r_sole"; 
        }
        //Translation Component
        if (! get_TF("chest", m_root_link_name))  // updates the TF variable with the most recent tf
        {
            throw std::logic_error("Cannot get TF 1");
        }
        m_odom_tf.transform.translation.x = in_bottle.get(0).asFloat64() - m_initial_offset_x - m_TF.transform.translation.x;
        m_odom_tf.transform.translation.y = in_bottle.get(1).asFloat64() - m_initial_offset_y - m_TF.transform.translation.y;
        m_odom_tf.transform.translation.z = in_bottle.get(2).asFloat64() + m_TF.transform.translation.z; 
        //Orientation
        if (! get_TF(m_foot_link, m_root_link_name))  // updates the TF variable with the most recent tf
        {
            throw std::logic_error("Cannot get TF 2");
        }
        tf2::Quaternion tf_ground;    //quat of the root_link to chest frame tf
        tf2::fromMsg(m_TF.transform.rotation, tf_ground);
        //Conversion from quat to rpy -> possible computational errors due to matricies
        tf2::Matrix3x3 m(tf_ground);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        tf2::Quaternion q_final;  //imu quaternion reading from sensor
        //q_final.setRPY(roll, pitch, -imu_bottle.get(3).asList()->get(0).asList()->get(0).asList()->get(1).asFloat64() * deg_to_rad);  //the real IMU will not have yaw orientation available
        //Handle yaw periodicity with the offset (I want to stay inside -Pi and +Pi range)
        if (in_bottle.get(5).asFloat64() - m_initial_offset_yaw > M_PI)
        {
            q_final.setRPY(roll, pitch, in_bottle.get(5).asFloat64() - m_initial_offset_yaw - M_PI);
        }
        else if (in_bottle.get(5).asFloat64() - m_initial_offset_yaw < -M_PI)
        {
            q_final.setRPY(roll, pitch, in_bottle.get(5).asFloat64() - m_initial_offset_yaw + M_PI);  //get yaw only from estimator
        }
        else
        {
            q_final.setRPY(roll, pitch, in_bottle.get(5).asFloat64() - m_initial_offset_yaw);  //get yaw only from estimator
        }
        //RCLCPP_INFO(this->get_logger(), "ESTIMATOR ROLL: %f PITCH: %f YAW: %f \n", in_bottle.get(3).asFloat64(), in_bottle.get(4).asFloat64(), in_bottle.get(5).asFloat64());

        m_odom_tf.transform.rotation.x = q_final.x();
        m_odom_tf.transform.rotation.y = q_final.y();
        m_odom_tf.transform.rotation.z = q_final.z();
        m_odom_tf.transform.rotation.w = q_final.w();

        // odom msg
        m_odom_msg.pose.pose.orientation = m_odom_tf.transform.rotation;
        m_odom_msg.pose.pose.position.x = m_odom_tf.transform.translation.x;
        m_odom_msg.pose.pose.position.y = m_odom_tf.transform.translation.y;
        m_odom_msg.pose.pose.position.z = m_odom_tf.transform.translation.z;
        m_odom_msg.twist.twist.linear.x = in_bottle.get(6).asFloat64();
        //m_odom_msg.twist.twist.linear.y = in_bottle.get(7).asFloat64();
        m_odom_msg.twist.twist.linear.y = 0;
        m_odom_msg.twist.twist.linear.z = in_bottle.get(8).asFloat64();
        m_odom_msg.twist.twist.angular.x = in_bottle.get(9).asFloat64();
        m_odom_msg.twist.twist.angular.y = in_bottle.get(10).asFloat64();
        m_odom_msg.twist.twist.angular.z = in_bottle.get(11).asFloat64();
        
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

bool Estimator_odom::publish_odom()
{
    try
    {
        //std::cout << "sending TF \n";
        //m_tf_broadcaster->sendTransform(m_odom_tf);
        //std::cout << "sending odom msg \n";
        m_odom_pub->publish(m_odom_msg);
        return true;
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "%s \n",e.what());
        return false;
    }
}

void Estimator_odom::timer_callback()
{
    //std::cout << "timer_callback exe \n";
    if(this->compute_odom())
    {
        if(!this->publish_odom()) {RCLCPP_WARN(this->get_logger(), "Failed in publishing odom \n");}
    }
    else{ RCLCPP_ERROR(this->get_logger(), "Failed in computing odom \n");  }
    //std::cout << "end callback \n";
}
