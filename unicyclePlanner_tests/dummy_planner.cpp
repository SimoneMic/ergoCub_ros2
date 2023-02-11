#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <chrono>
#include <memory>
using namespace std::chrono_literals;

class DummyPlanner : public rclcpp::Node
{
private:
    const std::string m_topic_name = "/plan"; 
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_pathPub;
    rclcpp::TimerBase::SharedPtr timer_;
    bool m_publishOnce = true;
    void timer_callback()
    {
        nav_msgs::msg::Path msg;
        msg.header.stamp = now();
        msg.header.frame_id = "virtual_unicycle_base";  //virtual_unicycle_base
        double y_increment = 0.05, x_increment = 0.05;
        for (int i = 0; i < 10; i++)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = msg.header.frame_id;
            pose.header.stamp = msg.header.stamp;
            pose.pose.position.x = i * x_increment;
            pose.pose.position.y = i * y_increment;
            pose.pose.position.z = 0.0;
            if (i ==0)
            {
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;
            }
            else
            {
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;
            }
            
            msg.poses.push_back(pose);
        }

        m_pathPub->publish(msg);
    };

public:
    DummyPlanner() : rclcpp::Node("dummy_planner")
    {   
        m_pathPub = this->create_publisher<nav_msgs::msg::Path>(m_topic_name, 10);
        timer_ = this->create_wall_timer(
        5000ms, std::bind(&DummyPlanner::timer_callback, this));

    }
};  //End of class PathConverter


int main(int argc, char** argv)
{
    // Init ROS2
    rclcpp::init(argc, argv);

    // Start listening in polling
    if (rclcpp::ok())
    {
        auto node = std::make_shared<DummyPlanner>();
        rclcpp::spin(node);
    }
    std::cout << "Shutting down" << std::endl;
    rclcpp::shutdown();
    return 0;
}