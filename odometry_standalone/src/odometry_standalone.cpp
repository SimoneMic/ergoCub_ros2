#include <odometry_standalone.hpp>

using std::placeholders::_1;

using namespace std::chrono_literals;

// Default constructor: creates ports and connects them
Estimator_odom::Estimator_odom()
: Node("odometry_node")
{
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);

    // YARP Port Connection
    odom_reader_port.open(reader_port_name);
    imu_reader_port.open(imu_reader_port_name);
    printf("Trying to connect to %s\n", writer_port);
    yarp::os::Network::connect(writer_port, reader_port_name);
    yarp::os::Network::connect(imu_writer_port, imu_reader_port_name);

    //std::cout << "created YARP ports \n";

    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_buffer_in = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_in);

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
    //std::cout << "creating wall timer \n";
    timer_ = this->create_wall_timer( 33ms, std::bind(&Estimator_odom::timer_callback, this));

    //std::cout << "created estimator odom object \n";
}

bool Estimator_odom::get_TF(const char* target_link)
{
    try
    {
        while (rclcpp::ok())
        {
            try
            {
                //std::cout << "looking for transform \n";
                TF = tf_buffer_in->lookupTransform(target_link, root_link_name, rclcpp::Time(0), 300ms); // target link = chest rclcpp::Time(0)
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
        std::cout << "compute_odom() \n";
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
        //std::cout << "getting TF \n";
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
        //std::cout << "computing quaternion \n";
        q.setRPY(0, 0, imu_bottle.get(3).asList()->get(0).asList()->get(0).asList()->get(2).asFloat64()* 0.0174533);
        new_quat = q * tf_quat;

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
        odom_msg.header.stamp = now(); // yarp::os::Time::now()
        odom_tf.header.stamp = odom_msg.header.stamp;
        //std::cout << "retunrning odom \n";
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
        //std::cout << "sending TF \n";
        tf_broadcaster->sendTransform(odom_tf);
        //std::cout << "sending odom msg \n";
        odom_pub->publish(odom_msg);
        return true;
    }
    catch(const std::exception& e)
    {
        RCLCPP_WARN(this->get_logger(), "%s \n",e.what());
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
    else{ RCLCPP_WARN(this->get_logger(), "Failed in computing odom \n");  }
    //std::cout << "end callback \n";
}
