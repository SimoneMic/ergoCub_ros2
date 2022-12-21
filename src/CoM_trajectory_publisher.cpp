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
    const std::string m_yarp_trajectory_port = "/planned_CoM/data:o";
    const std::string m_port_name = "/CoM_trajectory_publisher_test/reader:i";
    yarp::os::Port m_reader_port;

    const std::string m_topic_name = "/CoM_planned_trajectory_test";

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_downsampled_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_left_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_right_pub_;
    rclcpp::TimerBase::SharedPtr m_timer_;

    void callback()
    {
        //Feet pos tracker and notable points
        nav_msgs::msg::Path left_extremes;
        nav_msgs::msg::Path right_extremes;
        nav_msgs::msg::Path extremes_points;   
        nav_msgs::msg::Path center_points;
        //Std
        yarp::os::Bottle msg_in;
        nav_msgs::msg::Path CoM_path;
        nav_msgs::msg::Path downsampled_CoM_path;
        CoM_path.header.stamp = now();
        CoM_path.header.frame_id = "odom";      //odom
        downsampled_CoM_path.header = CoM_path.header;
        try
        {
            m_reader_port.read(msg_in);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << std::endl;
            return;
        }
        int n = msg_in.size();
        //RCLCPP_INFO(this->get_logger(), "Planned CoM trajectory (%i):\n", n);
        geometry_msgs::msg::PoseStamped pose_tmp;
        pose_tmp.header.stamp = now();
        pose_tmp.header.frame_id = "odom";
        tf2::Quaternion q;

        //Variables for finding local extremes on the left and rigt of the CoM path
        geometry_msgs::msg::PoseStamped max_pose;   //right extreme
        max_pose.pose.position.z = 0.0;
        max_pose.header = pose_tmp.header;
        geometry_msgs::msg::PoseStamped min_pose;   //left extreme
        min_pose.pose.position.z = 0.0;
        min_pose.header = pose_tmp.header;
        //Interpolated Centers of the CoM trajectory
        geometry_msgs::msg::PoseStamped interpolated_center;   //left extreme
        interpolated_center.pose.position.z = 0.0;
        interpolated_center.header = pose_tmp.header;
        //Loop through the CoM message (x_pose1 y_pose1 tetha_pose1 x_pose2 y_pose2 ...)
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
            //DOWNSAMPLE the CoM path
            if (downsampled_CoM_path.poses.empty())     //is it the first pose?
            {
                downsampled_CoM_path.poses.push_back(pose_tmp);
                //notable points -> start pose
                center_points.poses.push_back(pose_tmp);
                extremes_points.poses.push_back(pose_tmp);
            }
            else if (i+3 == n && i!=0)                   //is it the last pose?
            {
                downsampled_CoM_path.poses.push_back(pose_tmp);
                center_points.poses.push_back(pose_tmp);
                extremes_points.poses.push_back(pose_tmp);
            }
            else
            {
                //Distance check for downsampling
                double poses_distance = sqrt(std::pow(downsampled_CoM_path.poses.back().pose.position.x - CoM_path.poses.back().pose.position.x, 2) + 
                                        std::pow(downsampled_CoM_path.poses.back().pose.position.y - CoM_path.poses.back().pose.position.y, 2));
                if (poses_distance >= 0.01 )
                {
                    downsampled_CoM_path.poses.push_back(pose_tmp);
                }

                //Check for notable poses: extremes
                geometry_msgs::msg::PoseStamped prev_pose;  //could use a float or double
                //prev_pose.pose.position.x = msg_in.get(i-3).asFloat64();
                prev_pose.pose.position.y = msg_in.get(i-2).asFloat64();
                //How many positions are considered? check a neighbourhood of 2 - 3 - 5 poses?
                //Only on Y axes (horizontal)?
                //RIGHT EXTREMES
                if ((pose_tmp.pose.position.y - prev_pose.pose.position.y) > 0)
                {
                    max_pose.pose.position = pose_tmp.pose.position;
                }
                else 
                {
                    right_extremes.poses.push_back(max_pose);
                }

                //LEFT EXTREMES
                if ((pose_tmp.pose.position.y - prev_pose.pose.position.y) < 0)
                {
                    min_pose.pose.position = pose_tmp.pose.position;
                }
                else 
                {
                    left_extremes.poses.push_back(min_pose);
                }
            }

            //Once I've found the extremes I look for the points on the median between two consecutive extremes
            //Y position gives the order on what foot went first
            if (!left_extremes.poses.empty() && !right_extremes.poses.empty())
            {
                double interpolated_center_y = right_extremes.poses.back().pose.position.y - left_extremes.poses.back().pose.position.y;
                double interpolated_center_x = right_extremes.poses.back().pose.position.x - left_extremes.poses.back().pose.position.x;
            }
        }

        //RCLCPP_INFO(this->get_logger(), "Downsized CoM trajectory (%i) \n", downsampled_CoM_path.poses.size());
        m_pub_->publish(CoM_path);
        m_downsampled_pub_->publish(downsampled_CoM_path);
        m_left_pub_->publish(left_extremes);
        m_right_pub_->publish(right_extremes);
    }

public:
    CoM_trajectory_publisher(): Node("CoM_trajectory_publisher")
    {
        m_reader_port.open(m_port_name);
        yarp::os::Network::connect(m_yarp_trajectory_port, m_port_name);
        m_pub_ = this->create_publisher<nav_msgs::msg::Path>(m_topic_name, 10);
        m_downsampled_pub_ = this->create_publisher<nav_msgs::msg::Path>(m_topic_name + "_downsampled", 10);
        m_left_pub_ = this->create_publisher<nav_msgs::msg::Path>(m_topic_name + "_left", 10);
        m_right_pub_ = this->create_publisher<nav_msgs::msg::Path>(m_topic_name + "_right", 10);
        auto duration = std::chrono::duration<double>(1/10.0);
        m_timer_ = this->create_wall_timer(duration, std::bind(& CoM_trajectory_publisher::callback, this));
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
        std::cerr << 'rclcpp not ok' << std::endl;
        return 1;
    }
    std::cout << "Shutting down" << std::endl;
    rclcpp::shutdown();
    return 0;
}