
#include "yarp/os/Bottle.h"
#include "yarp/os/Port.h"
#include "yarp/os/BufferedPort.h"
#include "yarp/os/Network.h"
#include "yarp/os/RpcClient.h"
#include "yarp/sig/Vector.h"
#include "yarp/os/ConnectionWriter.h"
#include "yarp/os/Portable.h"

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_2d_msgs/msg/pose2_d_stamped.hpp"
#include "nav_2d_msgs/msg/path2_d.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <list>
#include <vector>
#include <mutex>

using namespace std::chrono_literals;
using std::placeholders::_1;

//Config file path: /home/user1/robotology-superbuild/build/install/share/ICUBcontrib/contexts/dcmWalknigNavigation/navigation.ini

// walkingNavigation path: 

//command for feet wrenches merge
//yarp merge --input /wholeBodyDynamics/right_foot_front/cartesianEndEffectorWrench:o /wholeBodyDynamics/left_foot_front/cartesianEndEffectorWrench:o --output /feetWrenches

//Class used for YARP port callbacks. Monitors the feet contacts state.
class YarpFeetDataProcessor : public yarp::os::PortReader
{
private:
    /* data */
    nav_msgs::msg::Path m_untransformed_path;
    yarp::os::BufferedPort<yarp::sig::VectorOf<double>> m_port;
    int m_sample_count = 0;
    /* Consts*/
    const double m_sensor_threshold = 90.0;     //100.0
    const std::string m_port_name = "/navigation_hsp/path:o";
    const std::string m_server_name = "/walking-coordinator/goal:i";

    /* Vars*/
    bool m_send_goal, m_step_check;
    bool m_initialized = false;  //used for passing tf reference objects
    bool send_once = true;  //for debug - used to send only one path
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;   //shared tf buffer
    std::mutex m_mutex;
    bool m_stop_cmd = false;    //flag for an external stop command
    bool m_path_completed = false;

    geometry_msgs::msg::PoseStamped transformPlan(geometry_msgs::msg::TransformStamped & tf_)
    {
        geometry_msgs::msg::PoseStamped goal_;
        if (m_untransformed_path.poses.empty()) {
            std::cerr << "Received plan with zero length" << std::endl;
            throw std::runtime_error("Received plan with zero length");
        }
        goal_.header.stamp = m_untransformed_path.header.stamp;
        goal_.header.frame_id = "projection";
        if (m_untransformed_path.poses.size() <= 2)
        {
            // Unitary path (?)
            tf2::doTransform(m_untransformed_path.poses.at(m_untransformed_path.poses.size()-1), goal_, tf_);
            std::cout << "Transformed X: " << goal_.pose.position.x << "Transformed Y: " << goal_.pose.position.y <<std::endl;
            m_path_completed = true;
        }
        else
        {
            //Take the 3rd element (the first one is the robot position itself)
            tf2::doTransform(m_untransformed_path.poses.at(3), goal_, tf_);
            std::cout << "Transformed X: " << goal_.pose.position.x << "Transformed Y: " << goal_.pose.position.y <<std::endl;
            //remove the first pose of the vector
            m_untransformed_path.poses.erase(m_untransformed_path.poses.begin());
        }
        return goal_;
    }

public:
    YarpFeetDataProcessor()
    {
        m_send_goal = false;
        m_step_check = true; //init to true to send at least the first command
        //Connection to the walking-controller goal port
        m_port.open(m_port_name);
        yarp::os::Network::connect(m_port_name, m_server_name);     
    };

    void init(std::shared_ptr<tf2_ros::Buffer> &buffer)
    {
        m_tf_buffer = buffer;
        m_initialized = true;
    }

    void storeSetpoint(const nav_msgs::msg::Path::ConstPtr &msg)
    {
        m_untransformed_path = *msg;
        m_path_completed = false;
    }

    //set the internal flag to whether send the goal or not
    void set_permission(bool val)
    {
        m_send_goal = val;
    }

    void stop_status(bool val)
    {
        m_stop_cmd = val;
    }

    bool read(yarp::os::ConnectionReader& connection) override
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        yarp::os::Bottle b;
        bool ok = b.read(connection);
        if (!ok) {
            std::cout << "Bad Yarp connection " << std::endl;
            return false;
        }
        
        //Condition for double feet support -> Should also check if in the middle of the CoM path during oscillation???
        if (b.get(2).asFloat64() > m_sensor_threshold && b.get(8).asFloat64() > m_sensor_threshold)
        {
            m_sample_count = 0;
            //std::cout << "Double Support " << std::endl;
            //This means that I have made a new step and either I have a new path, to finish one or a stop cmd to be executed (todo move stop cmd)
            if (m_step_check && m_send_goal && ( !m_path_completed))  
            {
                try
                {
                    if (m_initialized)
                    {
                        // STOP Given by a 0.0 0.0 0.0 velocity command
                        //if (m_stop_cmd)
                        //{
                        //    std::cout << "STOPPING ..." << std::endl;
                        //    auto& out = m_port.prepare();
                        //    out.clear();
                        //    out.push_back(0.0);
                        //    out.push_back(0.0);
                        //    m_port.write();
                        //    return true;    //exit condition
                        //}

                        // Extract the first transformed pose from the path to pass to the walking-controller
                        geometry_msgs::msg::TransformStamped TF = m_tf_buffer->lookupTransform("projection", m_untransformed_path.header.frame_id, rclcpp::Time(0), 50ms);
                        TF.transform.translation.x += 0.1;  //offsetted reference point used by the walking-controller -> found in config file by person distance
                        geometry_msgs::msg::PoseStamped goal = transformPlan(TF);

                        // Publish goal
                        std::cout << "Creating port buffer" << std::endl;
                        auto& out = m_port.prepare();
                        std::cout << "Clearing port buffer" << std::endl;
                        out.clear();
                        out.push_back(goal.pose.position.x);
                        out.push_back(goal.pose.position.y);
                        std::cout << "Passing element: X : " << out[0] << " Y: " << out[1] << std::endl;
                        std::cout << "Writing port buffer" << std::endl;
                        m_port.write();  

                        m_step_check = false;   //send data only once per double support
                    }
                    else
                    {
                        std::cout << "yarp::os::ConnectionReader not initialized: ROS TF buffer not passed" << std::endl;
                    }
                }
                catch(const std::exception& e)
                {
                    std::cerr << "Exception: " << e.what() << '\n';
                }
            }
        }
        else 
        {
            if (!m_step_check)
            {
                ++ m_sample_count;
                if (m_sample_count > 10)
                {
                    m_step_check = true;
                    std::cout << "Full Swing " << std::endl;
                }
            }
        }
        return true;
    }
    
};  //End of class YarpFeetDataProcessor : public yarp::os::PortReader


//This class subscribes to the /path topic and will pass it to the YarpFeetDataProcessor object
class PathConverter : public rclcpp::Node
{
private:
    const double zero_speed_threshold = 1e-03;      //should read it from .yaml params or ros parameters
    const std::string m_topic_name = "/plan";  // topic for TEB: /local_plan - for DWB: /plan
    const std::string m_vel_topic = "/cmd_vel";   
    yarp::os::Port m_feet_state_port;
    const std::string m_feet_state_port_name = "/setopint_converter/feet_state:i";
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr m_setpoint_sub;
    YarpFeetDataProcessor m_processor;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_velocity_sub;

    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer_in;

    //Each time I have a new path -i.e. a new path is published - I pass it and set the flag to publish it as soon as the requirements are met.
    void msg_callback(const nav_msgs::msg::Path::ConstPtr& msg_in)
    {
        m_processor.storeSetpoint(msg_in);
        m_processor.set_permission(true);
    }

    void vel_callback(const geometry_msgs::msg::Twist::ConstPtr & msg_in)
    {
        if ( std::abs(msg_in->linear.x) <= zero_speed_threshold && std::abs(msg_in->linear.y) <= zero_speed_threshold && std::abs(msg_in->angular.z) <= zero_speed_threshold)
        {
            m_processor.stop_status(true);
            RCLCPP_INFO(this->get_logger(), "DETECTED STOP COMMAND");
        }
        else
        {
            m_processor.stop_status(false);
        }
    }
public:
    PathConverter() : rclcpp::Node("path_converter_node")
    {   
        m_setpoint_sub = this->create_subscription<nav_msgs::msg::Path>(
            m_topic_name,
            10,
            std::bind(&PathConverter::msg_callback, this, _1)
        );
        m_velocity_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            m_vel_topic, 
            10, 
            std::bind(&PathConverter::vel_callback, this, _1)
        );
        // TFs
        m_tf_buffer_in = std::make_shared<tf2_ros::Buffer>(this->get_clock());    //share to the yarp port
        m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_in);
       
        //Create YarpFeetDataProcessor object
        m_feet_state_port.open(m_feet_state_port_name);
        yarp::os::Network::connect("/feetWrenches", m_feet_state_port_name);   //base-estimator/contacts/stateAndNormalForce:o
        if(yarp::os::Network::isConnected("/feetWrenches", m_feet_state_port_name)){
            RCLCPP_INFO(this->get_logger(), "YARP Ports connected successfully");
            m_feet_state_port.setReader(m_processor);
            m_processor.init(m_tf_buffer_in);
        } else {
            RCLCPP_ERROR(this->get_logger(), "[YARP] /feetWrenches NOT PRESENT:\n execute the merge command on the feet wrenches:\n yarp merge --input /wholeBodyDynamics/right_foot_front/cartesianEndEffectorWrench:o /wholeBodyDynamics/left_foot_front/cartesianEndEffectorWrench:o --output /feetWrenches");
        }
    }
};  //End of class PathConverter


int main(int argc, char** argv)
{
    // Init ROS2
    rclcpp::init(argc, argv);
    //Init YARP
    yarp::os::Network yarp;

    // Start listening in polling
    if (rclcpp::ok())
    {
        auto node = std::make_shared<PathConverter>();
        rclcpp::spin(node);
    }
    std::cout << "Shutting down" << std::endl;
    rclcpp::shutdown();
    return 0;
}