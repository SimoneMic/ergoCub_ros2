#include <estimator_odom.hpp>

using std::placeholders::_1;

// Default constructor: creates ports and connects them
Estimator_odom::Estimator_odom()
: Node("odometry_node")
{
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);

    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    chest_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
                                chest_imu_topic, 
                                default_qos, 
                                std::bind(&Estimator_odom::imu_callback, this, _1));

    //tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    //tfBuffer_in = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    xy_offsets_computed = false;
    yaw_offsets_computed = false;
    initial_offset_x = 0;
    initial_offset_y = 0;
    initial_offset_yaw = 0;
    imu_yaw = 0;
    /* set up odom msg */
    odom_msg.header.frame_id = odom_frame_name;
    odom_msg.child_frame_id = root_link_name;
    /* set up odom frame */
    odom_tf.child_frame_id = root_link_name;
    odom_tf.header.frame_id = odom_frame_name;
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

    initialized = false;
}

bool Estimator_odom::init()
{
    try
    {
        // YARP Port Connection
        odom_reader_port.open(reader_port_name);
        imu_reader_port.open(imu_reader_port_name);
        printf("Trying to connect to %s\n", writer_port);
        yarp::os::Network::connect(writer_port, reader_port_name);
        yarp::os::Network::connect(imu_writer_port, imu_reader_port_name);
        tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        // ROS 
        //chest_imu_sub = nh.subscribe(chest_imu_topic, 1, &Estimator_odom::imu_callback, this);
        initialized = true;
        return true;
    }
    catch(const std::exception& e)
    {
        RCLCPP_WARN(this->get_logger(), "%s \n",e.what());
        return false;
    }
}

bool Estimator_odom::get_TF(const char* target_link)
{
    try
    {
        if(! initialized)
        {
            throw std::logic_error(std::string("object not initialized: call init() method \n"));
        }
        while (rclcpp::ok())
        {
            try
            {
                TF = tfBuffer_in->lookupTransform(target_link, root_link_name, rclcpp::Time(0)); // target link = chest
                return true;
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "%s \n",ex.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(100));  //rclcpp::Duration(0.1).sleep();
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

bool Estimator_odom::compute_odom()
{
    try
    {
        Bottle in_bottle;
        odom_reader_port.read(in_bottle);
        Bottle imu_bottle;
        imu_reader_port.read(imu_bottle);
        if (! xy_offsets_computed)
        {
            initial_offset_x = in_bottle.get(0).asFloat64();
            initial_offset_y = in_bottle.get(1).asFloat64();
            xy_offsets_computed = true;
        }
        if (! get_TF("chest"))  // updates the TF variable with the most recent tf
        {
            throw std::logic_error("Cannot get TF ");
        }

        // TF Translation
        odom_tf.transform.translation.x = in_bottle.get(0).asFloat64() - initial_offset_x - TF.transform.translation.x;
        odom_tf.transform.translation.y = in_bottle.get(1).asFloat64() - initial_offset_y - TF.transform.translation.y;
        odom_tf.transform.translation.z = in_bottle.get(2).asFloat64() + TF.transform.translation.z;
        // TF Rotation
        tf2::Quaternion new_quat;   //imu quat in root_link frame
        tf2::Quaternion tf_quat;    //quat of the chest frame
        tf2::fromMsg(TF.transform.rotation, tf_quat);
        tf2::Quaternion q;  //temp
        //q.setRPY(0, 0, imu_yaw);
        q.setRPY(0, 0, imu_bottle.get(3).asList()->get(0).asList()->get(0).asList()->get(2).asFloat64()* 0.0174533);
        new_quat = q * tf_quat;
        //new_quat = tf_quat * q;
        //new_quat.normalize();
        odom_tf.transform.rotation.x = new_quat.x();
        odom_tf.transform.rotation.y = new_quat.y();
        odom_tf.transform.rotation.z = new_quat.z();
        odom_tf.transform.rotation.w = new_quat.w();

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
        odom_msg.header.stamp = rclcpp::Time(yarp::os::Time::now());
        odom_tf.header.stamp = odom_msg.header.stamp;
        return true;
    }
    catch(const std::exception& e)
    {
        RCLCPP_WARN(this->get_logger(), "%s \n",e.what());
        return false;
    }
}

bool Estimator_odom::publish_odom()
{
    try
    {
        tf_broadcaster->sendTransform(odom_tf);
        odom_pub->publish(odom_msg);
        return true;
    }
    catch(const std::exception& e)
    {
        RCLCPP_WARN(this->get_logger(), "%s \n",e.what());
        return false;
    }
}

void Estimator_odom::imu_callback(const sensor_msgs::msg::Imu::SharedPtr &msg)
{
    tf2::fromMsg(msg->orientation, imu_quat);
    tf2::Matrix3x3 m(imu_quat);
    double r, p ;
    m.getRPY(r, p, imu_yaw);
    //if (! yaw_offsets_computed)
    //{
    //    initial_offset_yaw = imu_yaw;
    //    yaw_offsets_computed = true;
    //}
    //imu_yaw = imu_yaw - initial_offset_yaw;
    //4 debug
    Eigen::Quaterniond q;
    tf2::fromMsg(msg->orientation, q);
    auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
    Bottle imu_bottle;
    imu_reader_port.read(imu_bottle);
    RCLCPP_INFO(this->get_logger(), "YARP VALUE: %f vs Eigen: %f vs ROS: %f \n", imu_bottle.get(3).asList()->get(0).asList()->get(0).asList()->get(2).asFloat64()* 0.0174533
            , euler(2)
            , imu_yaw);

}

int main(int argc, char* argv[])
{
    // YARP network
    yarp::os::Network yarp;
    // ROS
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Estimator_odom>;
    Estimator_odom odom_obj;
    if (! odom_obj.init())
    {
        RCLCPP_WARN(odom_obj.get_logger(), "Failed initializing odometry. Closing node. \n");
        return 1;
    }

    double const loopFreq = 50.0;
    rclcpp::Rate looprate(loopFreq);

    while (rclcpp::ok()) 
    {
        rclcpp::spin_some(std::make_shared<Estimator_odom>());
        odom_obj.compute_odom();
        odom_obj.publish_odom();
        looprate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}