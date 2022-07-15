#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

#include <yarp/os/Bottle.h>
#include <yarp/os/Port.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class ChestProjection : public rclcpp::Node
{
private:
    /* const */
    const char* reader_port_name = "/chest_projector/wrench_reader:i";
    const char* writer_port_name = "/base-estimator/contacts/stateAndNormalForce:o";
    const double loopFreq = 30.0;
    const char* chest_link = "chest";
    /* msgs */
    geometry_msgs::msg::TransformStamped TF;
    geometry_msgs::msg::TransformStamped projection_TF;
    /* TF objects*/
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_in;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    /* YARP ports*/
    yarp::os::Port wrench_reader_port;
    /* var */
    char* foot_link = "r_sole";
    //timer for loop
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();
    bool get_TF(const char* target_link, const char* source_link);

    //virtual unicycle base position variables
    double y_translation;
    geometry_msgs::msg::TransformStamped initial_tf_right;
    geometry_msgs::msg::TransformStamped initial_tf_left;
    geometry_msgs::msg::TransformStamped virtual_unicycle_base_tf;
    bool initial_state_computed;
public:
    ChestProjection();
};

ChestProjection::ChestProjection() : rclcpp::Node("chest_projection_node")
{
    wrench_reader_port.open(reader_port_name);
    yarp::os::Network::connect(writer_port_name, reader_port_name);

    foot_link = "r_sole";
    projection_TF.child_frame_id = "projection";
    /* init timer*/
    auto duration = std::chrono::duration<double>(1/loopFreq);
    timer_ = this->create_wall_timer( duration , std::bind(&ChestProjection::timer_callback, this));
    /* init TFs*/
    tf_pub = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_buffer_in = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_in);

    initial_state_computed = false;
    virtual_unicycle_base_tf.child_frame_id = "virtual_unicycle_base";
}

void ChestProjection::timer_callback()
{
    // Check which foot is on the ground based on the wrenches
    yarp::os::Bottle in_bottle;
    wrench_reader_port.read(in_bottle);
    //Initial state computation when the robot is standing still in double support
    if (!initial_state_computed)
    {
        //check for double support
        if (in_bottle.get(0).asFloat64() > 100.0 && in_bottle.get(1).asFloat64() > 100.0)
        {
            //get initial TFs from chest to each foot
            // Get TF of chest in right sole
            if (!get_TF("r_sole", chest_link))
            {
                RCLCPP_ERROR(this->get_logger(), "Cannot find TF between %s and %s \n", chest_link, "r_sole");
            }
            else
            {
                //TF got right
                initial_tf_right = TF;
                //Get TF of chest in left sole
                if (!get_TF("l_sole", chest_link))
                {
                    RCLCPP_ERROR(this->get_logger(), "Cannot find TF between %s and %s \n", chest_link, "l_sole");
                }
                else
                {
                    //BOTH TF got right
                    initial_tf_left = TF;
                    initial_state_computed = true;
                }
            }
        }
    }
    //determine which feet is in contact
    if (in_bottle.get(0).asFloat64() > 100.0)
    {
        foot_link = "l_sole";   //r_sole
        projection_TF.header.frame_id = foot_link;

        virtual_unicycle_base_tf.header.frame_id = foot_link;
    }
    else if (in_bottle.get(1).asFloat64() > 100.0)
    {
        foot_link = "r_sole";   //l_sole
        projection_TF.header.frame_id = foot_link;

        virtual_unicycle_base_tf.header.frame_id = foot_link;
    }
    // Get TF
    if (!get_TF(chest_link, foot_link))
    {
        RCLCPP_WARN(this->get_logger(), "Cannot find TF between \n");
    }
    // Got TF
    // Compute the projection to the same ground of the fixed foot
    // get RPY
    tf2::Quaternion tf_quat;
    tf2::fromMsg(TF.transform.rotation, tf_quat);
    tf2::Matrix3x3 m(tf_quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    //tf_quat.setRPY(roll, pitch, 0.0);
    tf_quat.setRPY(0, 0, yaw);
    //projection_TF.transform.translation.x = 0;
    //projection_TF.transform.translation.y = 0;
    //projection_TF.transform.translation.z = TF.transform.translation.z;
    projection_TF.transform.translation.x = -TF.transform.translation.x;
    projection_TF.transform.translation.y = -TF.transform.translation.y;
    projection_TF.transform.translation.z = 0;
    projection_TF.transform.rotation = tf2::toMsg(tf_quat);
    //Calculation of virtual unicycle base position tf
    virtual_unicycle_base_tf.header.stamp = projection_TF.header.stamp = now();
    virtual_unicycle_base_tf.transform.translation.x = projection_TF.transform.translation.x;
    if (foot_link=="r_sole")
    {
        get_TF(foot_link, "l_sole");
        virtual_unicycle_base_tf.transform.translation.y = TF.transform.translation.y/2;
    }
    else
    {
        get_TF(foot_link, "r_sole");
        virtual_unicycle_base_tf.transform.translation.y = TF.transform.translation.y/2;
    }
    virtual_unicycle_base_tf.transform.translation.z = 0;
    virtual_unicycle_base_tf.transform.rotation = projection_TF.transform.rotation;
    std::vector<geometry_msgs::msg::TransformStamped> tf_buffer;
    tf_buffer.push_back(projection_TF);
    tf_buffer.push_back(virtual_unicycle_base_tf);
    tf_pub->sendTransform(tf_buffer);
}

// transform the data originated in the source_link to the frame expressed by the target_link
bool ChestProjection::get_TF(const char* target_link, const char* source_link)
{
    try
    {
        while (rclcpp::ok())
        {
            try
            {
                //std::cout << "looking for transform \n";
                TF = tf_buffer_in->lookupTransform(target_link, source_link, rclcpp::Time(0), 100ms); // target link = chest
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

int main(int argc, char* argv[])
{
    // YARP init
    yarp::os::Network yarp;
    // ROS
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChestProjection>();   
    if (rclcpp::ok()) 
    {
        std::cout << "Spinning chest_projection node \n";
        rclcpp::spin(node);
    }
    return 0;
}