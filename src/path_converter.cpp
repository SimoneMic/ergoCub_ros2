
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
    nav_msgs::msg::Path::ConstPtr m_untransformed_path;
    yarp::os::BufferedPort<yarp::sig::VectorOf<double>> m_port;
    yarp::os::Port m_rpc_port;
    /* Consts*/
    const double m_sensor_threshold = 100.0;
    const std::string m_port_name = "/navigation_hsp/path:o";
    const std::string m_server_name = "/navigation/path:i";
    const std::string m_rpc_server_name = "/navigation/rpc";
    const std::string m_rpc_client_name = "/path_converter/rpc";

    /* Vars*/
    bool m_send_goal, m_step_check;
    bool m_initialized = false;  //used for passing tf reference objects
    bool send_once = true;  //for debug - used to send only one path
    bool m_first_time = true; //used for replanning
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;   //shared tf buffer
    std::mutex m_mutex;
    bool m_stop_cmd = false;    //flag for an external stop command

    nav_msgs::msg::Path transformPlan(geometry_msgs::msg::TransformStamped & tf_)
    {
        if (m_untransformed_path->poses.empty()) {
            std::cerr << "Received plan with zero length" << std::endl;
            throw std::runtime_error("Received plan with zero length");
        }
        
        // let's get the pose of the robot in the frame of the plan

        // Transform the near part of the global plan into the robot's frame of reference.
        nav_msgs::msg::Path transformed_plan_ = *m_untransformed_path;
        transformed_plan_.header.frame_id = "projection";
        transformed_plan_.header.stamp = m_untransformed_path->header.stamp;
        // Helper function for the transform below. Converts a pose2D from global
        // frame to local
        //auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
        //    nav_2d_msgs::msg::Pose2DStamped stamped_pose, transformed_pose;
        //    stamped_pose.header.frame_id = global_plan_.header.frame_id;
        //    stamped_pose.pose = global_plan_pose;
        //    nav_2d_utils::transformPose(
        //      tf_, transformed_plan.header.frame_id,
        //      stamped_pose, transformed_pose, transform_tolerance_);
        //    return transformed_pose.pose;
        //  };
        
        //std::transform(
        //    transformation_begin, transformation_end,
        //    std::back_inserter(transformed_plan.poses),
        //    transformGlobalPoseToLocal);

        //Transform the whole path
        //std::cout << "Transform the whole path for loop" << std::endl;
        for (int i = 0; i < m_untransformed_path->poses.size(); ++i)
        {
            tf2::doTransform(m_untransformed_path->poses.at(i), transformed_plan_.poses.at(i), tf_);
            std::cout << "Transformed X: " << transformed_plan_.poses.at(i).pose.position.x << "Transformed Y: " << transformed_plan_.poses.at(i).pose.position.y <<std::endl;
        }
        // Remove the portion of the global plan that we've already passed so we don't
        // process it on the next iteration.
        //if (prune_plan_) {
        //  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
        //  pub_->publishGlobalPlan(global_plan_);
        //}
        
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
        m_port.open(m_port_name);
        yarp::os::Network::connect(m_port_name, m_server_name);     
        //rpc connection
        std::cout << "Connecting RPC ..." << std::endl;
        m_rpc_port.open(m_rpc_client_name);
        yarp::os::Network::connect(m_rpc_client_name, m_rpc_server_name);
        if(yarp::os::Network::isConnected(m_rpc_client_name, m_rpc_server_name)){
            std::cout << "Ports Connected" << std::endl;
        } else {
            std::cout << m_rpc_server_name << " NOT PRESENT" << std::endl;
        }
        
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
            //check for data sanity before transmitting it
            //todo
            if (m_send_goal && m_step_check)  //This means that I have a new path to send
            {
                try
                {
                    if (m_initialized)
                    {
                        if (m_stop_cmd)
                        {
                            std::cout << "STOPPING ..." << std::endl;
                            yarp::os::Bottle cmd, response;
                            if(!m_first_time)
                            {
                                cmd.addString("replan");
                                m_rpc_port.write(cmd, response);
                                if (!response.get(0).asBool())
                                {
                                    std::cerr << "Replanning command sent but not received! " << response.get(0).asInt64() << std::endl;
                                }
                                else{
                                    std::cout << "Replanning command received: " << response.get(0).asInt64() << std::endl;
                                }    
                            }
                            std::cout << "Creating port buffer" << std::endl;
                            //Convert Path to yarp vector
                            auto& out = m_port.prepare();
                            std::cout << "Clearing port buffer" << std::endl;
                            out.clear();
                            out.push_back(0.0);
                            out.push_back(0.0);
                            
                            m_port.write();  //send data only once per double support

                            cmd.clear();
                            response.clear();
                            cmd.addString("persist");
                            m_rpc_port.write(cmd, response);
                            
                            if (!response.get(0).asBool())
                            {
                                std::cerr << "Persist command sent but not received! " << response.get(0).asInt64() << std::endl;
                            }
                            else
                            {
                                std::cout << "Persist command received " << response.get(0).asInt64() << std::endl;
                            } 
                            return true;
                        }

                        // FOR DEBUG ONLY SEND ONE PATH
                        if (!m_first_time && !m_stop_cmd)
                            {
                                yarp::os::Bottle cmd, response;
                                cmd.clear();
                                cmd.addString("persist");
                                m_rpc_port.write(cmd, response);
                                if (!response.get(0).asBool())
                                {
                                    std::cerr << "[DEBUG RETURN] Persist command sent but not received!" << std::endl;
                                }
                                else
                                {
                                    std::cout << "[DEBUG RETURN] Persist command received" << std::endl;
                                } 
                                return true;
                            }

                        geometry_msgs::msg::TransformStamped TF = m_tf_buffer->lookupTransform("projection", m_untransformed_path->header.frame_id, rclcpp::Time(0), 50ms);
                        TF.transform.translation.x += 0.1;  //offsetted reference point used by the walking-controller -> found in config file by person distance
                        nav_msgs::msg::Path transformed_plan = transformPlan(TF);
                        if (transformed_plan.poses.size()>0)
                        {
                            // RPC port
                            yarp::os::Bottle cmd, response;
                            if(!m_first_time)
                            {
                                std::cout << "Replanning ..." << std::endl;
                                cmd.addString("replan");
                                m_rpc_port.write(cmd, response);
                                if (!response.get(0).asBool())
                                {
                                    std::cerr << "Replanning command sent but not received! " << response.get(0).asInt64() << std::endl;
                                }
                                else{
                                    std::cout << "Replanning command received: " << response.get(0).asInt64() << std::endl;
                                }    
                            }
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
                            
                            cmd.clear();
                            response.clear();
                            cmd.addString("persist");
                            m_rpc_port.write(cmd, response);
                            
                            if (!response.get(0).asBool())
                            {
                                std::cerr << "Persist command sent but not received! " << response.get(0).asInt64() << std::endl;
                            }
                            else
                            {
                                std::cout << "Persist command received " << response.get(0).asInt64() << std::endl;
                            } 
                            m_first_time = false;  
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
            else
            {
                if (!m_first_time && !m_stop_cmd)
                {
                    yarp::os::Bottle cmd, response;
                    cmd.clear();
                    cmd.addString("persist");
                    m_rpc_port.write(cmd, response);
                    if (!response.get(0).asBool())
                    {
                        std::cerr << "[Outer if] Persist command sent but not received!" << std::endl;
                    }
                    else
                    {
                        std::cout << "[Outer if] Persist command received" << std::endl;
                    } 
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
class PathConverter : public rclcpp::Node
{
private:
    const double zero_speed_threshold = 1e-03;
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