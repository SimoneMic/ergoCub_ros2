#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "yarp/os/Bottle.h"
#include "yarp/os/Network.h"
#include "yarp/os/Port.h"

#include <string>
#include <chrono>

class CoM_trajectory_publisher : public rclcpp::Node
{
private:
    const std::string yarp_trajectory_port = "/planned_CoM/data:o";
    const std::string port_name = "/CoM_trajectory_publisher/reader:i";
    yarp::os::Port reader_port;

    const std::string topic_name = "/CoM_planned_trajectory";

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void callback()
    {
        yarp::os::Bottle msg_in;
        reader_port.read(msg_in);
        int n = msg_in.size();
        RCLCPP_INFO(this->get_logger(), "Planned CoM trajectory (%i):\n", n);
        for(int i=0; i<n ; i+=3)
        {
            geometry_msgs::msg::Point point;
            for(int j=0; j<2; ++j)
            {
                if(j==0)
                    point.x = msg_in.get(i+j).asFloat64();
                else if(j==1)
                    point.y = msg_in.get(i+j).asFloat64();
                else
                    point.z = msg_in.get(i+j).asFloat64();  //z is theta
            }
            RCLCPP_INFO(this->get_logger(), "(%f %f %f)\n", point.x, point.y, point.z);
        }
        RCLCPP_INFO(this->get_logger(), "-----------------------------------\n");
    }

public:
    CoM_trajectory_publisher(): Node("CoM_trajectory_publisher")
    {
        reader_port.open(port_name);
        yarp::os::Network::connect(yarp_trajectory_port, port_name);
        pub_ = this->create_publisher<nav_msgs::msg::Path>(topic_name, 10);
        auto duration = std::chrono::duration<double>(1.0);
        timer_ = this->create_wall_timer(duration, std::bind(& CoM_trajectory_publisher::callback, this));
    }
    
};



int main(int argc, char ** argv)
{
    yarp::os::Network yarp;
    if (rclcpp::ok())
    {
        auto node = std::make_shared<CoM_trajectory_publisher>();
        rclcpp::spin(node);
    }
    else
    {
        return 1;
    }
    
    return 0;
}