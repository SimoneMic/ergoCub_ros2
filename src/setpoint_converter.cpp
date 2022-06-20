#include "yarp/os/Bottle.h"
#include "yarp/os/Port.h"
#include "yarp/os/Network.h"
#include "yarp/os/RpcClient.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <list>

using std::placeholders::_1;

class SetpointConverter : public rclcpp::Node
{
private:
    const char* port_name = "/setopint_converter/talker:o";
    const char* server_name = "/walking-coordinator/goal:i";
    const double x_vel_min = 0.04;
    const double x_vel_max = 0.2;
    const double y_vel_min = 0.05;
    const double y_vel_max = 0.6;
    yarp::os::Port port;
    
    double setpoint_x = 0;
    double setpoint_y = 0;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr setpoint_sub_;
    

    void msg_callback(const geometry_msgs::msg::Twist::ConstPtr& msg_in)
    {
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
public:
    SetpointConverter() : rclcpp::Node("setpoint_converter_node")
    {
        // Network Setup
        yarp::os::Network yarp;
        port.open(port_name);
        yarp::os::Network::connect(port_name, server_name);     //todo - check for connection or errors
        setpoint_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel",
            10,
            std::bind(&SetpointConverter::msg_callback, this, _1)
        );
    }
};






int main(int argc, char** argv)
{
    // Init ROS2
    rclcpp::init(argc, argv);

    // Start listening in polling
    if (rclcpp::ok())
    {
        auto node = std::make_shared<SetpointConverter>();
        rclcpp::spin(node);
    }
    
    return 0;
}