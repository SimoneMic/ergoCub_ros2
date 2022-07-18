#include "yarp/os/Bottle.h"
#include "yarp/os/Port.h"
#include "yarp/os/Network.h"
#include "yarp/os/RpcClient.h"

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <list>

using namespace std::chrono_literals;
using std::placeholders::_1;

//Class used for YARP port callbacks
class YarpFeetDataProcessor : public yarp::os::PortReader
{
private:
    /* data */
    yarp::os::Bottle transformed_goal_;
    bool on_double_support;     //flag for when the robot enters on double support
    yarp::os::Port port;

    const double sensor_threshold = 100.0;

    const std::string port_name = "/setopint_converter/talker:o";
    const std::string server_name = "/walking-coordinator/goal:i";
public:
    YarpFeetDataProcessor()
    {
        on_double_support = false;

        port.open(port_name);
        yarp::os::Network::connect(port_name, server_name);     //todo - check for connection or errors
    };

    void storePath(yarp::os::Bottle &msg)
    {
        transformed_goal_ = msg;
    }

    bool read(yarp::os::ConnectionReader& connection) override
    {
        yarp::os::Bottle b;
        bool ok = b.read(connection);
        if (!ok) {
            //std::cout<<
            return false;
        }
        //Condition for double support -> Should also check if in the middle of the CoM path during oscillation???
        if (b.get(0).asFloat64() > sensor_threshold && b.get(1).asFloat64() > sensor_threshold)
        {
            //if I wasn't on double support before I can send data -> this means that it's the first time
            if (!on_double_support)
            {
                on_double_support = true;

                //check for data sanity before transmitting it
                //todo
                port.write(transformed_goal_);  //send data only once per double support
            }
        }
        else
        {
            on_double_support = false;
        }
        return true;
    }
};


class SetpointConverter : public rclcpp::Node
{
private:
    const std::string topic_name = "/plan";     //subscribe to global plan
    //const char* port_name = "/setopint_converter/talker:o";
    //const char* server_name = "/walking-coordinator/goal:i";
    //const double x_vel_min = 0.04;
    //const double x_vel_max = 0.2;
    //const double y_vel_min = 0.05;
    //const double y_vel_max = 0.6;
    //yarp::os::Port port;
    yarp::os::Port feet_state_port;
    const std::string feet_state_port_name = "/setopint_converter/feet_state:i";
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_in;
    
    //double setpoint_x = 0;
    //double setpoint_y = 0;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr setpoint_sub_;
    YarpFeetDataProcessor processor;
    /*
    void msg_callback(const geometry_msgs::msg::Twist::ConstPtr& msg_in)
    {
        //SPEED CONVERTER
        // X
        if (msg_in->linear.x >= x_vel_min && msg_in->linear.x <= x_vel_max)    
            //setpoint_x = msg_in->linear.x * 5;
            setpoint_x = 1;
        else if (msg_in->linear.x > 0 && msg_in->linear.x < x_vel_min)
            setpoint_x = 0.2;
        else if (msg_in->linear.x > x_vel_max)
            setpoint_x = 1;
        else
            setpoint_x = 0;     //for safety

        // Y
        if (abs(msg_in->angular.z) >= y_vel_min && abs(msg_in->angular.z) <= y_vel_max)
            setpoint_y =  msg_in->angular.z * 2;
        else if (abs(msg_in->angular.z) > 0 && abs(msg_in->angular.z) < y_vel_min)
            setpoint_y =  0.1 * (msg_in->angular.z / abs(msg_in->angular.z));
        else if (abs(msg_in->angular.z) > y_vel_max)
            setpoint_y = (msg_in->angular.z / abs(msg_in->angular.z));
        else
            setpoint_y = 0;     //for safety

        yarp::os::Bottle msg_out;
        msg_out.addFloat64(setpoint_x);
        msg_out.addFloat64(setpoint_y);
        RCLCPP_INFO(this->get_logger(), "Converting cmd_vel from x: %f yaw: %f ---> x: %f y: %f \n", msg_in->linear.x, msg_in->angular.z, setpoint_x, setpoint_y);

        port.write(msg_out);
    }
    */

   void msg_callback(const nav_msgs::msg::Path::ConstPtr& msg_in)
    {

        yarp::os::Bottle msg_out;
        // Need to transform the path from map frame to robot frame
        geometry_msgs::msg::PoseStamped goal_robot;
        //tf_buffer_in->transform(msg_in->poses.back(), goal_robot, "chest", 200ms);    //goal pose in robot frame: could throw exception for extrapolation into the future
        try
        {
            //
            geometry_msgs::msg::TransformStamped  TF = tf_buffer_in->lookupTransform("root_link", msg_in->header.frame_id, rclcpp::Time(0), 100ms);
            TF.transform.translation.x += 0.1;  //offsetted reference point used by the walking-controller
            //todo: implement a slicing of the path
            tf2::doTransform(msg_in->poses.back(), goal_robot, TF);
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform Exception: %s \n", e.what());
        }
        msg_out.addFloat64(goal_robot.pose.position.x);
        msg_out.addFloat64(goal_robot.pose.position.y);
        RCLCPP_INFO(this->get_logger(), "Setpoint x: %f y: %f \n", msg_out.get(0).asFloat64(), msg_out.get(1).asFloat64());
        //I should publish the command only when the robot is in double support -> other callback
        //port.write(msg_out);
        processor.storePath(msg_out);
    }
public:
    SetpointConverter() : rclcpp::Node("setpoint_converter_node")
    {
        // Network Setup
        //yarp::os::Network yarp;
        
        setpoint_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            topic_name,
            10,
            std::bind(&SetpointConverter::msg_callback, this, _1)
        );

        tf_buffer_in = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_in);

        //Create YarpFeetDataProcessor object
        feet_state_port.open(feet_state_port_name);
        yarp::os::Network::connect("/base-estimator/contacts/stateAndNormalForce:o", feet_state_port_name);
        feet_state_port.setReader(processor);
    }
};


int main(int argc, char** argv)
{
    // Init ROS2
    rclcpp::init(argc, argv);
    //Init YARP
    yarp::os::Network yarp;

    // Start listening in polling
    if (rclcpp::ok())
    {
        auto node = std::make_shared<SetpointConverter>();
        rclcpp::spin(node);
    }
    
    return 0;
}