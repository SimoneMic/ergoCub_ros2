
#include "yarp/os/Bottle.h"
#include "yarp/os/Port.h"
#include "yarp/os/BufferedPort.h"
#include "yarp/os/Network.h"
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

#include <list>
#include <vector>
#include <mutex>

using namespace std::chrono_literals;
using std::placeholders::_1;

//command for feet wrenches merge
//yarp merge --input /wholeBodyDynamics/right_foot_front/cartesianEndEffectorWrench:o /wholeBodyDynamics/left_foot_front/cartesianEndEffectorWrench:o --output /feetWrenches

//Class used for YARP port callbacks. Monitors the feet contacts state.
class YarpFeetDataProcessor : public yarp::os::PortReader
{
private:
    /* data */
    nav_msgs::msg::Path::ConstPtr m_untransformed_path;
    yarp::os::BufferedPort<yarp::sig::VectorOf<double>> m_port;
    /* Consts*/
    const double m_sensor_threshold = 100.0;
    const std::string m_outPortName = "/path_converter/path:o";
    const std::string m_inPortName = "/walking-coordinator/goal:i";

    /* Vars*/
    bool m_send_goal;   //flag set externally when a new path has been published
    bool m_step_check;  //flag that signals the fact that a new step has been made
    bool m_initialized = false;  //check if the object has been initialized - used for passing tf reference objects
    bool send_once = true;  //for debug - used to send only one path
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;   //shared tf buffer
    std::mutex m_mutex;

    nav_msgs::msg::Path transformPlan(geometry_msgs::msg::TransformStamped & t_tf, bool t_prune = true)
    {
        if (m_untransformed_path->poses.empty()) {
            std::cerr << "Received plan with zero length" << std::endl;
            throw std::runtime_error("Received plan with zero length");
        }

        // Transform the global plan into the robot's frame of reference.
        nav_msgs::msg::Path transformed_plan_ = *m_untransformed_path;
        transformed_plan_.header.frame_id = "virtual_unicycle_base";   //virtual_unicycle_base
        transformed_plan_.header.stamp = m_untransformed_path->header.stamp;

        //Transform the whole path
        //std::cout << "Transform the whole path for loop" << std::endl;
        for (int i = 0; i < m_untransformed_path->poses.size(); ++i)
        {
            tf2::doTransform(m_untransformed_path->poses.at(i), transformed_plan_.poses.at(i), t_tf);
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
    YarpFeetDataProcessor()
    {
        m_send_goal = false;
        m_step_check = true; //init to true to send at least the first command
        //Connection to the walking-controller goal port
        m_port.open(m_outPortName);
        yarp::os::Network::connect(m_outPortName, m_inPortName);             
    };

    void init(std::shared_ptr<tf2_ros::Buffer> &buffer)
    {
        m_tf_buffer = buffer;
        m_initialized = true;
    }

    void storeSetpoint(const nav_msgs::msg::Path::ConstPtr &msg)
    {
        m_untransformed_path = msg;
    }

    //set the internal flag to whether send the goal or not
    void set_permission(bool val)
    {
        m_send_goal = val;
    }

    //main loop executed for each port reading of the merged feet status
    bool read(yarp::os::ConnectionReader& t_connection) override
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        yarp::os::Bottle b;
        bool ok = b.read(t_connection);
        if (!ok) {
            std::cout << "Bad Yarp connection " << std::endl;
            return false;
        }
        
        //Condition for double feet support -> Should also check if in the middle of the CoM path during oscillation???
        if (b.get(2).asFloat64() > m_sensor_threshold && b.get(8).asFloat64() > m_sensor_threshold)
        {
            if (m_send_goal && m_step_check)  //This means that I have a new path to send and a new step made
            {
                try
                {
                    if (m_initialized)
                    {
                        geometry_msgs::msg::TransformStamped TF = m_tf_buffer->lookupTransform("projection", m_untransformed_path->header.frame_id, rclcpp::Time(0), 50ms);
                        TF.transform.translation.x += 0.1;  //offsetted reference point used by the walking-controller -> found in config file by person distance
                        nav_msgs::msg::Path transformed_plan = transformPlan(TF, true);
                        if (transformed_plan.poses.size()>0)
                        {
                            std::cout << "Creating port buffer" << std::endl;
                            //Convert Path to yarp vector
                            auto& out = m_port.prepare();
                            std::cout << "Clearing port buffer" << std::endl;
                            out.clear();
                            for (int i = 0; i < transformed_plan.poses.size(); ++i)
                            {
                                out.push_back(transformed_plan.poses.at(i).pose.position.x);
                                out.push_back(transformed_plan.poses.at(i).pose.position.y);
                                std::cout << "Passing Path i-th element: " << i << " X : " << out[2*i] << " Y: " << out[2*i+1] << std::endl;
                            }
                            // Persist command tells the walking controller to actuate the plan
                            
                            std::cout << "Writing port buffer" << std::endl;
                            m_port.write();  //send data only once per double support
                             
                            m_send_goal = false;
                            m_step_check = false;
                            //send_once = false;  //only for debug
                        }
                        else
                        {
                            std::cout << "Returned empty transformed path" << std::endl;
                        }
                    }
                    else
                    {
                        std::cout << "Failed to transform path" << std::endl;
                    }
                    //if I have a stop command, I should still be able to send a new one: I have two commands per double support
                    //todo ?
                }
                catch(const std::exception& e)
                {
                    std::cerr << "Exception in Transform Path: " << e.what() << '\n';
                }
            }
        }
        else {
            m_step_check = true;
        }
        return true;
    }
    
};  //End of class YarpFeetDataProcessor : public yarp::os::PortReader


//This class subscribes to the /path topic and will pass it to the YarpFeetDataProcessor object
class WalkingControllerTalker : public rclcpp::Node
{
private:
    const double zero_speed_threshold = 1e-03;
    const std::string m_topic_name = "/plan";    
    yarp::os::Port m_feet_state_port;
    const std::string m_outPortName = "/path_converter/path:o";
    const std::string m_inPortName = "/walking-coordinator/goal:i";
    nav_msgs::msg::Path::ConstPtr m_untransformed_path;
    yarp::os::BufferedPort<yarp::sig::VectorOf<double>> m_port;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr m_setpoint_sub;
    YarpFeetDataProcessor m_processor;

    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer_in;

    //Each time I have a new path -i.e. a new path is published - I pass it and set the flag to publish it as soon as the requirements are met.
    void msg_callback(const nav_msgs::msg::Path::ConstPtr& msg_in)
    {
        m_processor.storeSetpoint(msg_in);
        m_processor.set_permission(true);
    }

public:
    WalkingControllerTalker() : rclcpp::Node("path_converter_node")
    {   
        m_setpoint_sub = this->create_subscription<nav_msgs::msg::Path>(
            m_topic_name,
            10,
            std::bind(&WalkingControllerTalker::msg_callback, this, _1)
        );

        // TFs
        m_tf_buffer_in = std::make_shared<tf2_ros::Buffer>(this->get_clock());    //share to the yarp port
        m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_in);
       
        //Create YarpFeetDataProcessor object
        m_port.open(m_outPortName);
        yarp::os::Network::connect(m_outPortName, m_inPortName);   
        if(yarp::os::Network::isConnected(m_outPortName, m_inPortName)){
            RCLCPP_INFO(this->get_logger(), "YARP Ports connected successfully");
            m_feet_state_port.setReader(m_processor);
            m_processor.init(m_tf_buffer_in);
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