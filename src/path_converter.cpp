
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
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <list>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;

//Config file path: /home/user1/robotology-superbuild/build/install/share/ICUBcontrib/contexts/dcmWalknigNavigation/navigation.ini

//command for feet wrenches merge
//yarp merge --input /wholeBodyDynamics/right_foot_front/cartesianEndEffectorWrench:o /wholeBodyDynamics/left_foot_front/cartesianEndEffectorWrench:o --output /feetWrenches

//Class used for YARP port callbacks. Monitors the feet contacts state.
class YarpFeetDataProcessor : public yarp::os::PortReader
{
private:
    /* data */
    nav_msgs::msg::Path::ConstPtr untransformed_path;
    yarp::sig::VectorOf<double> goal_path;
    //yarp::os::Port m_port;
    yarp::os::BufferedPort<yarp::sig::VectorOf<double> > m_port;
    const double m_sensor_threshold = 100.0;
    const std::string m_port_name = "/path_converter/talker:o";
    const std::string m_server_name = "/path:i";
    bool m_send_goal, m_step_check;
    bool initialized = false;  //used for passing tf reference objects

    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;   //shared tf buffer
public:
    YarpFeetDataProcessor()
    {
        m_send_goal = false;
        m_step_check = true; //init to true to send at least the first command
        //Connection to the walking-controller goal port
        m_port.open(m_port_name);
        yarp::os::Network::connect(m_port_name, m_server_name);     //todo - check for connection or errors
    };

    void init(std::shared_ptr<tf2_ros::Buffer> &buffer)
    {
        m_tf_buffer = buffer;
        initialized = true;
    }

    void storeSetpoint(const nav_msgs::msg::Path::ConstPtr &msg)
    {
        untransformed_path = msg;
    }

    //set the internal flag to whether send the goal or not
    void set_permission(bool val)
    {
        m_send_goal = val;
    }

    bool read(yarp::os::ConnectionReader& connection) override
    {
        yarp::os::Bottle b;
        bool ok = b.read(connection);
        if (!ok) {
            std::cout << "Bad Yarp connection \n";
            return false;
        }
        //Condition for double feet support -> Should also check if in the middle of the CoM path during oscillation???
        if (b.get(2).asFloat64() > m_sensor_threshold && b.get(8).asFloat64() > m_sensor_threshold)
        {
            
            //check for data sanity before transmitting it
            //todo
            if (m_send_goal & m_step_check)  //This means that I have a new path to send
            {
                try
                {
                    //Transform Path
                    nav_msgs::msg::Path transformed_path;
                    if (initialized)
                    {
                        geometry_msgs::msg::TransformStamped TF = m_tf_buffer->lookupTransform("root_link", untransformed_path->header.frame_id, rclcpp::Time(0), 50ms);
                        TF.transform.translation.x += 0.1;  //offsetted reference point used by the walking-controller -> found in config file by person distance
                        tf2::doTransform(*untransformed_path, transformed_path, TF);
                    }
                    else
                    {
                        /* code */
                    }
                    
                    //Convert Path to yarp vector
                    for (size_t i = 0; i < transformed_path.poses.size(); ++i)
                    {
                        goal_path.push_back(transformed_path.poses.at(i).pose.position.x);
                        goal_path.push_back(transformed_path.poses.at(i).pose.position.y);
                        std::cout << "Passing Path i-th element: " << i << " : " << transformed_path.poses.at(i).pose.position.x << " Y: " << transformed_path.poses.at(i).pose.position.y << std::endl;
                    }
                    auto& out = m_port.prepare();
                    out.clear();
                    out = goal_path;
                    m_port.write();  //send data only once per double support
                    m_send_goal = false;
                    //if I have a stop command, I should still be able to send a new one: I have two commands per double support
                    //todo?
                }
                catch(const std::exception& e)
                {
                    std::cerr << e.what() << '\n';
                }
            }
        }
        else
        {
            m_step_check = true;
        }
        
        return true;
    }
};  //End of class YarpFeetDataProcessor : public yarp::os::PortReader


//This class subscribes to the /path topic and will pass it to the YarpFeetDataProcessor object
class PathConverter : public rclcpp::Node
{
private:
    const std::string m_topic_name = "/local_plan";     //subscribe to cmd_vel
    yarp::os::Port m_feet_state_port;
    const std::string m_feet_state_port_name = "/setopint_converter/feet_state:i";
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
    PathConverter() : rclcpp::Node("path_converter_node")
    {   
        m_setpoint_sub = this->create_subscription<nav_msgs::msg::Path>(
            m_topic_name,
            10,
            std::bind(&PathConverter::msg_callback, this, _1)
        );
        // TFs
        m_tf_buffer_in = std::make_shared<tf2_ros::Buffer>(this->get_clock());    //share to the yarp port
        m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_in);
       
        //Create YarpFeetDataProcessor object
        m_feet_state_port.open(m_feet_state_port_name);
        yarp::os::Network::connect("/feetWrenches", m_feet_state_port_name);   //base-estimator/contacts/stateAndNormalForce:o
        m_feet_state_port.setReader(m_processor);
        m_processor.init(m_tf_buffer_in);
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