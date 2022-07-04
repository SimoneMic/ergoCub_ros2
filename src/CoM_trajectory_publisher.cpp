#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "yarp/os/Bottle.h"
#include "yarp/os/Network.h"
#include "yarp/os/Port.h"

#include <string>
#include <chrono>
#include <math.h>

class CoM_trajectory_publisher : public rclcpp::Node
{
private:
    const std::string yarp_trajectory_port = "/planned_CoM/data:o";
    const std::string port_name = "/CoM_trajectory_publisher/reader:i";
    yarp::os::Port reader_port;

    const std::string topic_name = "/CoM_planned_trajectory";

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr downsampled_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void callback()
    {
        yarp::os::Bottle msg_in;
        nav_msgs::msg::Path CoM_path;
        nav_msgs::msg::Path downsampled_CoM_path;
        CoM_path.header.stamp = now();
        CoM_path.header.frame_id = "odom";
        downsampled_CoM_path.header = CoM_path.header;
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
        RCLCPP_INFO(this->get_logger(), "Planned CoM trajectory (%i):\n", n);
        geometry_msgs::msg::PoseStamped pose_tmp;
        pose_tmp.header.stamp = now();
        pose_tmp.header.frame_id = "odom";
        tf2::Quaternion q;
        for(int i=0; i<n ; i+=3)
        {
            q.setRPY(0, 0, msg_in.get(i+2).asFloat64());
            pose_tmp.pose.orientation.w = q.w();
            pose_tmp.pose.orientation.x = q.x();
            pose_tmp.pose.orientation.y = q.y();
            pose_tmp.pose.orientation.z = q.z();
            pose_tmp.pose.position.x = msg_in.get(i).asFloat64();
            pose_tmp.pose.position.y = msg_in.get(i+1).asFloat64();
            CoM_path.poses.push_back(pose_tmp);
            //DOWNSAMPLE
            if (downsampled_CoM_path.poses.empty())     //is it the first pose?
            {
                downsampled_CoM_path.poses.push_back(pose_tmp);
            }
            else if (i+3 == n && i!=0)                   //is it the last pose?
            {
                downsampled_CoM_path.poses.push_back(pose_tmp);
            }
            else
            {
                double poses_distance = sqrt(std::pow(downsampled_CoM_path.poses.back().pose.position.x - CoM_path.poses.back().pose.position.x, 2) + 
                                        std::pow(downsampled_CoM_path.poses.back().pose.position.y - CoM_path.poses.back().pose.position.y, 2));
                if (poses_distance >= 0.01 )
                {
                    downsampled_CoM_path.poses.push_back(pose_tmp);
                }
            }
        }
        RCLCPP_INFO(this->get_logger(), "Downsized CoM trajectory (%i) \n", downsampled_CoM_path.poses.size());
        pub_->publish(CoM_path);
        downsampled_pub_->publish(downsampled_CoM_path);
    }

public:
    CoM_trajectory_publisher(): Node("CoM_trajectory_publisher")
    {
        reader_port.open(port_name);
        yarp::os::Network::connect(yarp_trajectory_port, port_name);
        pub_ = this->create_publisher<nav_msgs::msg::Path>(topic_name, 10);
        downsampled_pub_ = this->create_publisher<nav_msgs::msg::Path>(topic_name + "_downsampled", 10);
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