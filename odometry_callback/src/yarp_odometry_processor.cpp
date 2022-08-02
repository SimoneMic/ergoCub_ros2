#include <yarp_odometry_processor.hpp>

YarpOdometryProcessor::YarpOdometryProcessor()
{
    //Yarp Network & ROS initialized in the main file
    p_ros_ = std::make_shared<OdomPublisher>();
    //Init variables
    m_xy_offsets_computed = false;
    m_initial_offset_x = 0;
    m_initial_offset_y = 0;
    m_foot_link = "r_sole"; 
};

YarpOdometryProcessor::~YarpOdometryProcessor()
{};

bool YarpOdometryProcessor::read(yarp::os::ConnectionReader& connection)
{
    //std::cout << "start reading" << std::endl;
    //Init variables used in each thread
    nav_msgs::msg::Odometry odom_msg;
    tf2::Quaternion imu_quat;
    geometry_msgs::msg::TransformStamped tf, odom_tf;
    /* set up odom msg */
    odom_msg.header.frame_id = m_odom_frame_name;
    odom_msg.child_frame_id = m_root_link_name;
    /* set up odom frame */
    odom_tf.child_frame_id = m_root_link_name;
    odom_tf.header.frame_id = m_odom_frame_name;
    // TODO - Tune covariance matricies
    odom_msg.pose.covariance = {0.05, 0    , 0   , 0   , 0   , 0   , 
                                0   , 0.05 , 0   , 0   , 0   , 0   , 
                                0   , 0    , 0.02, 0   , 0   , 0   , 
                                0   , 0    , 0   , 0.06, 0   , 0   ,
                                0   , 0    , 0   , 0   , 0.06, 0   , 
                                0   , 0    , 0   , 0   , 0   , 0.06};
    odom_msg.twist.covariance = {0.1, 0    , 0   , 0   , 0   , 0   , 
                                0   , 0.1  , 0   , 0   , 0   , 0   , 
                                0   , 0    , 0.05, 0   , 0   , 0   , 
                                0   , 0    , 0   , 0.12, 0   , 0   ,
                                0   , 0    , 0   , 0   , 0.12, 0   , 
                                0   , 0    , 0   , 0   , 0   , 0.12};
    //Read congregated data from /odometry_state:o
    yarp::os::Bottle data;
    bool ok = data.read(connection);
    if (!ok) {
        std::cout << "Bad Yarp connection" << std::endl;
        return false;
    }
    try
    {
        //std::cout << "Initial offset" << std::endl;
        //Compute odom initial offsets on X Y components
        if (! m_xy_offsets_computed)
        {
            m_initial_offset_x = data.get(0).asFloat64();
            m_initial_offset_y = data.get(1).asFloat64();
            m_xy_offsets_computed = true;
        }
        //Find which foot is in contact with the ground -> for planarity
        if (data.get(22).asFloat64() > m_sensor_threshold && data.get(23).asFloat64() < m_sensor_threshold)
        {
            m_foot_link = "l_sole"; 
        }
        else if (data.get(22).asFloat64() < m_sensor_threshold && data.get(23).asFloat64() > m_sensor_threshold)
        {
            m_foot_link = "r_sole"; 
        }
        else if (data.get(22).asFloat64() < m_sensor_threshold && data.get(23).asFloat64() < m_sensor_threshold)
        {
            //TODO handle exception of robot not in contact with ground-> when in double support do nothing, keep the previous value
            //std::cout << "Robot not in contact with the ground" << std::endl;
        }
        //std::cout << "Translation Comp" << std::endl;
        //Translation Component of odometry X Y Z
        if (! p_ros_->get_TF("chest", m_root_link_name, tf))  // updates the TF variable with the most recent tf
        {
            throw std::logic_error("Cannot get TF 1");
        }
        odom_tf.transform.translation.x = data.get(0).asFloat64() - m_initial_offset_x - tf.transform.translation.x;
        odom_tf.transform.translation.y = data.get(1).asFloat64() - m_initial_offset_y - tf.transform.translation.y;
        odom_tf.transform.translation.z = data.get(2).asFloat64() + tf.transform.translation.z; 
        //Orientation
        //std::cout << "Orientation Comp" << std::endl;
        if (! p_ros_->get_TF(m_foot_link, m_root_link_name, tf))  // updates the TF variable with the most recent tf
        {
            throw std::logic_error("Cannot get TF 2");
        }
        tf2::Quaternion tf_ground;    //quat of the root_link to chest frame tf
        tf2::fromMsg(tf.transform.rotation, tf_ground);
        tf2::Matrix3x3 m(tf_ground);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        tf2::Quaternion q_final;  //final quaternion reading from imu sensor only the yaw component
        //q_final.setRPY(roll, pitch, -data.get(12).asList()->get(0).asList()->get(0).asList()->get(1).asFloat64()* deg_to_rad);
        q_final.setRPY(roll, pitch, data.get(5).asFloat64());

        odom_tf.transform.rotation.x = q_final.x();
        odom_tf.transform.rotation.y = q_final.y();
        odom_tf.transform.rotation.z = q_final.z();
        odom_tf.transform.rotation.w = q_final.w();
        // odom msg
        odom_msg.pose.pose.orientation = odom_tf.transform.rotation;
        odom_msg.pose.pose.position.x = odom_tf.transform.translation.x;
        odom_msg.pose.pose.position.y = odom_tf.transform.translation.y;
        odom_msg.pose.pose.position.z = odom_tf.transform.translation.z;
        odom_msg.twist.twist.linear.x = data.get(6).asFloat64();
        odom_msg.twist.twist.linear.y = data.get(7).asFloat64();
        odom_msg.twist.twist.linear.z = data.get(8).asFloat64();
        odom_msg.twist.twist.angular.x = data.get(9).asFloat64();
        odom_msg.twist.twist.angular.y = data.get(10).asFloat64();
        odom_msg.twist.twist.angular.z = data.get(11).asFloat64();

        // time stamps done when published
        //odom_msg.header.stamp = now(); // yarp::os::Time::now()
        //odom_tf.header.stamp = odom_msg.header.stamp;
        //std::cout << "Publishing odom" << std::endl;
        p_ros_->publish_odom(odom_tf, odom_msg);
    }
    catch(const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return false;
    }
    return true;
};