
#include "yarp/os/Bottle.h"
#include "yarp/os/Port.h"
#include "yarp/os/BufferedPort.h"
#include "yarp/os/Network.h"
#include "yarp/sig/Vector.h"
#include "yarp/os/ConnectionWriter.h"
#include "yarp/os/Portable.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_2d_msgs/msg/pose2_d_stamped.hpp"
#include "nav_2d_msgs/msg/path2_d.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <list>
#include <vector>
#include <mutex>

using namespace std::chrono_literals;
using std::placeholders::_1;

//This class subscribes to the /path topic and will pass it to the walking-controller on a yarp port
class WalkingControllerTalker : public rclcpp::Node
{
private:
    const double zero_speed_threshold = 1e-03;
    const std::string m_topic_name = "/plan";
    const std::string m_state_topic = "/is_goal_reached/goal_state";
    yarp::os::Port m_feet_state_port;
    const std::string m_outPortName = "/path_converter/path:o";
    const std::string m_inPortName = "/walking-coordinator/goal:i";
    const std::string m_reference_frame = "virtual_unicycle_base";
    yarp::os::BufferedPort<yarp::sig::VectorOf<double>> m_port;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr m_setpoint_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_state_sub;

    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::mutex m_mutex;
    bool m_goalReached = false;

    //Each time I have a new path we transform the path and pass it to the walking controller
    void msg_callback(const nav_msgs::msg::Path::ConstPtr& msg_in)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_goalReached)
        {
            geometry_msgs::msg::TransformStamped TF = m_tf_buffer->lookupTransform(m_reference_frame, msg_in->header.frame_id, rclcpp::Time(0));
            TF.transform.translation.x += 0.1;  //offsetted reference point used by the walking-controller -> found in config file by person distance
            nav_msgs::msg::Path transformed_plan = transformPlan(msg_in, TF, true);
            if (transformed_plan.poses.size()>0)
            {
                //std::cout << "Creating port buffer" << std::endl;
                //Convert Path to yarp vector
                auto& out = m_port.prepare();
                //std::cout << "Clearing port buffer" << std::endl;
                out.clear();
                for (int i = 0; i < transformed_plan.poses.size(); ++i)
                {
                    out.push_back(transformed_plan.poses.at(i).pose.position.x);
                    out.push_back(transformed_plan.poses.at(i).pose.position.y);
                    std::cout << "Passing Path i-th element: " << i << " X : " << out[2*i] << " Y: " << out[2*i+1] << std::endl;
                }

                //std::cout << "Writing port buffer" << std::endl;
                m_port.write();
            }
            else
            {
                std::cout << "Returned empty transformed path" << std::endl;
            }
        }
        else
        {
            //Write a path with all 0 poses
            auto& out = m_port.prepare();
            //std::cout << "Clearing port buffer" << std::endl;
            out.clear();
            for (int i = 0; i < 3; ++i)
            {
                out.push_back(0.0);
                out.push_back(0.0);
                std::cout << "Passing Path i-th element: " << i << " X : " << out[2*i] << " Y: " << out[2*i+1] << std::endl;
            }
            //std::cout << "Writing port buffer" << std::endl;
            m_port.write();
        }
    }

    //Each time I have a new path we transform the path and pass it to the walking controller
    void state_callback(const std_msgs::msg::Bool::ConstPtr& in)
    {
        if (in->data)
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_goalReached = true;
            //Write a path with all 0 poses
            auto& out = m_port.prepare();
            //std::cout << "Clearing port buffer" << std::endl;
            out.clear();
            for (int i = 0; i < 3; ++i)
            {
                out.push_back(0.0);
                out.push_back(0.0);
                std::cout << "Passing Path i-th element: " << i << " X : " << out[2*i] << " Y: " << out[2*i+1] << std::endl;
            }
            //std::cout << "Writing port buffer" << std::endl;
            m_port.write();
        }
        else
        {
            m_goalReached = false;
        }
    }

    nav_msgs::msg::Path transformPlan(const nav_msgs::msg::Path::ConstPtr& path, geometry_msgs::msg::TransformStamped & t_tf, bool t_prune = true)
    {
        if (path->poses.empty()) {
            std::cerr << "Received plan with zero length" << std::endl;
            throw std::runtime_error("Received plan with zero length");
        }

        // Transform the global plan into the robot's frame of reference.
        nav_msgs::msg::Path transformed_plan_ = *path;
        transformed_plan_.header.frame_id = "virtual_unicycle_base";   //virtual_unicycle_base
        transformed_plan_.header.stamp = path->header.stamp;

        //Transform the whole path
        //std::cout << "Transform the whole path for loop" << std::endl;
        for (int i = 0; i < path->poses.size(); ++i)
        {
            tf2::doTransform(path->poses.at(i), transformed_plan_.poses.at(i), t_tf);
            std::cout << "Transformed X: " << transformed_plan_.poses.at(i).pose.position.x << "Transformed Y: " << transformed_plan_.poses.at(i).pose.position.y <<std::endl;
        }
        // Remove the portion of the global plan that we've already passed so we don't -> i.e the one with negative X
        // process it on the next iteration. (can't be done in odom frame)
        if (t_prune && transformed_plan_.header.frame_id != "odom")
        {
            // Helper predicate lambda function to see what is the positive x-element in a vector of poses
            // Warning: The robot needs to have a portion of the path that goes forward (positive X)
            auto greaterThanZero = [](const geometry_msgs::msg::PoseStamped &i){
                return i.pose.position.x > 0.0;
            };

            transformed_plan_.poses.erase(begin(transformed_plan_.poses), 
                                        std::find_if(transformed_plan_.poses.begin(),
                                                transformed_plan_.poses.end(),
                                                greaterThanZero));
        }
        
        if (transformed_plan_.poses.empty()) {
            std::cerr << "Resulting plan has 0 poses in it." << std::endl;
            throw std::runtime_error("Resulting plan has 0 poses in it");
        }
        return transformed_plan_;
    }

public:
    WalkingControllerTalker() : rclcpp::Node("path_converter_node")
    {   
        m_setpoint_sub = this->create_subscription<nav_msgs::msg::Path>(
            m_topic_name,
            10,
            std::bind(&WalkingControllerTalker::msg_callback, this, _1)
        );

        m_state_sub = this->create_subscription<std_msgs::msg::Bool>(
            m_state_topic,
            10,
            std::bind(&WalkingControllerTalker::state_callback, this, _1)
        );

        // TFs
        m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());    //share to the yarp port
        m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
       
        //Create YarpFeetDataProcessor object
        m_port.open(m_outPortName);
        yarp::os::Network::connect(m_outPortName, m_inPortName);   
        if(yarp::os::Network::isConnected(m_outPortName, m_inPortName)){
            RCLCPP_INFO(this->get_logger(), "YARP Ports connected successfully");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Could not connect ports");
        }
    }
};  //End of class WalkingControllerTalker


int main(int argc, char** argv)
{
    // Init ROS2
    rclcpp::init(argc, argv);
    //Init YARP
    yarp::os::Network yarp;

    // Start listening in polling
    if (rclcpp::ok())
    {
        auto node = std::make_shared<WalkingControllerTalker>();
        rclcpp::spin(node);
    }
    std::cout << "Shutting down" << std::endl;
    rclcpp::shutdown();
    return 0;
}