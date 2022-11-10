#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "yarp/os/Network.h"
#include "yarp/os/Port.h"

#include <string>
#include <chrono>
#include <algorithm>
#include <mutex>
#include <cmath>
#include <memory>
#include <iostream>
#include <deque>

class FootstepPublisher : public rclcpp::Node
{
private:
    const std::string m_feetOutPort = "/walking-controller/feet_positions:o";
    const std::string m_portName = "/feet_port_listener/feet_positions:i";
    yarp::os::Port m_readerPort;

    const std::string m_ritghFootprintsTopicName = "/unicycle_path_follower/right_footprints";
    const std::string m_leftFootprintsTopicName = "/unicycle_path_follower/left_footprints";

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_rightFootprintMarkersPub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_leftFootprintMarkersPub;

    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer_in;

    std::mutex m_mutex;

public:
    FootstepPublisher(): Node("footstep_publisher")
    {
        m_readerPort.open(m_portName);
        yarp::os::Network::connect(m_feetOutPort, m_portName);
        m_rightFootprintMarkersPub = this->create_publisher<visualization_msgs::msg::MarkerArray>(m_ritghFootprintsTopicName, 10);
        m_leftFootprintMarkersPub = this->create_publisher<visualization_msgs::msg::MarkerArray>(m_leftFootprintsTopicName, 10);
    }

    bool publishMarkers(std::vector<std::vector<double>> t_leftSteps, std::vector<std::vector<double>> t_rightSteps){
        std::lock_guard(m_mutex);
        
        if (t_leftSteps.size()==0 || t_rightSteps.size()==0)
        {
            RCLCPP_INFO(this->get_logger(), "One of the Step array is empty");
            return false;
        }
        
        visualization_msgs::msg::MarkerArray right_marker_array;
        visualization_msgs::msg::MarkerArray left_marker_array;
        visualization_msgs::msg::Marker tmp_marker_msg;
        builtin_interfaces::msg::Time timestamp = now();
        //LEFT
        RCLCPP_INFO(this->get_logger(), "Left Loop");
        for (size_t i = 0; i < t_leftSteps.size(); ++i)
        {
            visualization_msgs::msg::Marker tmp_marker_msg;
            tmp_marker_msg.header.frame_id = "virtual_unicycle_base";   
            tmp_marker_msg.id = i;
            tmp_marker_msg.header.stamp = timestamp;
            tmp_marker_msg.scale.x = 0.05;
            tmp_marker_msg.scale.y = 0.05;
            tmp_marker_msg.scale.z = 0.05;
            // Color for left foot
            tmp_marker_msg.color.r = 0.0;
            tmp_marker_msg.color.g = 1.0;
            tmp_marker_msg.color.b = 0.0;
            tmp_marker_msg.color.a = 1.0;
            tmp_marker_msg.type = visualization_msgs::msg::Marker::ARROW;
            tmp_marker_msg.pose.position.x = t_leftSteps.at(i)[0];
            tmp_marker_msg.pose.position.y = t_leftSteps.at(i)[1];
            tmp_marker_msg.pose.position.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, t_leftSteps.at(i)[2]);
            tmp_marker_msg.pose.orientation = tf2::toMsg(q);
            tmp_marker_msg.frame_locked = true;
            tmp_marker_msg.action = visualization_msgs::msg::Marker::ADD;
            //Populate the marker with atleast one mesh point
            geometry_msgs::msg::Point cube_center;
            cube_center.x = 0.0;
            cube_center.y = 0.0;
            cube_center.z = 0.0;
            tmp_marker_msg.points.push_back(cube_center);
            cube_center.x = 0.1;
            cube_center.y = 0.0;
            cube_center.z = 0.0;
            tmp_marker_msg.points.push_back(cube_center);
            //save marker in the array
            left_marker_array.markers.push_back(tmp_marker_msg);
        }
        RCLCPP_INFO(this->get_logger(), "Publishing Left");
        m_leftFootprintMarkersPub->publish(left_marker_array);
        //for (auto it = leftSteps.begin(); it != leftSteps.end(); ++it)
        //{
        //    tmp_marker_msg.id = std::distance(leftSteps.begin(), it); 
        //    tmp_marker_msg.pose.position.x = it->position(0);
        //    tmp_marker_msg.pose.position.y = it->position(1);
        //    tf2::Quaternion q;
        //    q.setRPY(0, 0, it->angle);
        //    tmp_marker_msg.pose.orientation = tf2::toMsg(q);
//
        //    left_marker_array.markers.push_back(tmp_marker_msg);
        //}

        //RIGHT
        RCLCPP_INFO(this->get_logger(), "Right Loop");
        /*
        for (auto it = rightSteps.begin(); it != rightSteps.end(); ++it)
        {
            tmp_marker_msg.id = std::distance(rightSteps.begin(), it); 
            tmp_marker_msg.pose.position.x = it->position(0);
            tmp_marker_msg.pose.position.y = it->position(1);
            tf2::Quaternion q;
            q.setRPY(0, 0, it->angle);
            tmp_marker_msg.pose.orientation = tf2::toMsg(q);

            right_marker_array.markers.push_back(tmp_marker_msg);
        } */
        tmp_marker_msg.points.clear();
        for (size_t i = 0; i < t_rightSteps.size(); ++i)
        {
            visualization_msgs::msg::Marker tmp_marker_msg;
            tmp_marker_msg.header.frame_id = "virtual_unicycle_base";
            tmp_marker_msg.id = i;
            tmp_marker_msg.header.stamp = timestamp;
            tmp_marker_msg.scale.x = 0.05;
            tmp_marker_msg.scale.y = 0.05;
            tmp_marker_msg.scale.z = 0.05;
            // Color for left foot
            tmp_marker_msg.color.r = 1.0;
            tmp_marker_msg.color.g = 0.0;
            tmp_marker_msg.color.b = 0.0;
            tmp_marker_msg.color.a = 1.0;
            tmp_marker_msg.type = visualization_msgs::msg::Marker::ARROW;
            tmp_marker_msg.pose.position.x = t_rightSteps.at(i)[0];
            tmp_marker_msg.pose.position.y = t_rightSteps.at(i)[1];
            tmp_marker_msg.pose.position.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, t_rightSteps.at(i)[2]);
            tmp_marker_msg.pose.orientation = tf2::toMsg(q);
            tmp_marker_msg.frame_locked = true;
            tmp_marker_msg.action = visualization_msgs::msg::Marker::ADD;
            //Populate the marker with atleast 2 mesh point for ARROW
            geometry_msgs::msg::Point cube_center;
            cube_center.x = 0.0;
            cube_center.y = 0.0;
            cube_center.z = 0.0;
            tmp_marker_msg.points.push_back(cube_center);
            cube_center.x = 0.1;
            cube_center.y = 0.0;
            cube_center.z = 0.0;
            tmp_marker_msg.points.push_back(cube_center);
            //save marker in the array
            right_marker_array.markers.push_back(tmp_marker_msg);
        }

        RCLCPP_INFO(this->get_logger(), "Publishing Right");
        //publish
        m_rightFootprintMarkersPub->publish(right_marker_array);
    }
};

int main(int argc, char ** argv)
{
    yarp::os::Network yarp;
    rclcpp::init(argc, argv);
    if (rclcpp::ok())
    {
        auto node = std::make_shared<FootstepPublisher>();
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