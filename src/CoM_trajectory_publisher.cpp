#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

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
        nav_msgs::msg::Path path;
        path.header.stamp = now();
        path.header.frame_id = "odom";
        try
        {
            reader_port.read(msg_in);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            return;
        }
        int n = msg_in.size();
        //RCLCPP_INFO(this->get_logger(), "Planned CoM trajectory (%i):\n", n);
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = now();
        pose.header.frame_id = "odom";
        tf2::Quaternion q;
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
            q.setRPY(0, 0, point.z);
            pose.pose.orientation.w = q.w();
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.position.x = point.x;
            pose.pose.position.y = point.y;
            pose.pose.position.z = 0;
            path.poses.push_back(pose);

            //RCLCPP_INFO(this->get_logger(), "(%f %f %f)\n", point.x, point.y, point.z);
        }
        pub_->publish(path);
        //RCLCPP_INFO(this->get_logger(), "-----------------------------------\n");
    }

public:
    CoM_trajectory_publisher(): Node("CoM_trajectory_publisher")
    {
        reader_port.open(port_name);
        yarp::os::Network::connect(yarp_trajectory_port, port_name);
        pub_ = this->create_publisher<nav_msgs::msg::Path>(topic_name, 10);
        auto duration = std::chrono::duration<double>(1.0);
        timer_ = this->create_wall_timer(duration, std::bind(& CoM_trajectory_publisher::callback, this));
        RCLCPP_INFO(this->get_logger(), "Created Node\n");
    }
};

int main(int argc, char ** argv)
{
    yarp::os::Network yarp;
    rclcpp::init(argc, argv);
    if (rclcpp::ok())
    {
        auto node = std::make_shared<CoM_trajectory_publisher>();
        rclcpp::spin(node);
    }
    else
    {
        std::cerr << 'rclcpp not ok\n';
        return 1;
    }
    return 0;
}