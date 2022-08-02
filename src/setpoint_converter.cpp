#include "yarp/os/Bottle.h"
#include "yarp/os/Port.h"
#include "yarp/os/Network.h"
#include "yarp/os/RpcClient.h"

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <list>

using namespace std::chrono_literals;
using std::placeholders::_1;

//Class used for YARP port callbacks. Monitors the feet contacts state.
class YarpFeetDataProcessor : public yarp::os::PortReader
{
private:
    /* data */
    nav_msgs::msg::Path::ConstPtr m_path_msg;
    yarp::os::Port m_port;
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
    const double m_sensor_threshold = 100.0;
    const std::string m_port_name = "/setopint_converter/talker:o";
    const std::string m_server_name = "/walking-coordinator/goal:i";
    bool m_send_goal;
public:
    YarpFeetDataProcessor()
    {
        m_send_goal = false;
        //Connection to the walking-controller goal port
        m_port.open(m_port_name);
        yarp::os::Network::connect(m_port_name, m_server_name);     //todo - check for connection or errors
    };

    void init(std::shared_ptr<tf2_ros::Buffer> &buffer)
    {
        m_tf_buffer = buffer;
    }

    void storePath(const nav_msgs::msg::Path::ConstPtr& msg)
    {
        m_path_msg = msg;
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
        if (b.get(0).asFloat64() > m_sensor_threshold && b.get(1).asFloat64() > m_sensor_threshold)
        {
            //check for data sanity before transmitting it
            //todo
            if (m_send_goal)  //This means that I have a new path
            {
                //Delay -> wait to be in the middle of the double support
                std::this_thread::sleep_for(500ms);
                //I have to transform the goal to the robot reference frame (root_link) only when I have to send the data
                yarp::os::Bottle msg_out;
                geometry_msgs::msg::PoseStamped goal_robot;
                goal_robot.pose.position.x = 0;
                goal_robot.pose.position.y = 0;
                try
                {
                    geometry_msgs::msg::TransformStamped  TF = m_tf_buffer->lookupTransform("root_link", m_path_msg->header.frame_id, rclcpp::Time(0), 50ms);
                    TF.transform.translation.x += 0.1;  //offsetted reference point used by the walking-controller -> found in config file by person distance
                    //todo: implement a slicing of the path
                    tf2::doTransform(m_path_msg->poses.back(), goal_robot, TF);
                }
                catch(const std::exception& e)
                {
                    std::cerr << "Transform Exception: " << e.what() << "\n";
                    //if there's an error, the robot will stop
                    goal_robot.pose.position.x = 0;
                    goal_robot.pose.position.y = 0;
                }
                msg_out.addFloat64(goal_robot.pose.position.x);
                msg_out.addFloat64(goal_robot.pose.position.y);
                std::cout << "Passing Setpoint x: " << msg_out.get(0).asFloat64() << " y: " << msg_out.get(1).asFloat64() << "\n";
                m_port.write(msg_out);  //send data only once per double support
                m_send_goal = false;
            }
        }
        return true;
    }
};  //End of class YarpFeetDataProcessor : public yarp::os::PortReader


//This class subscribes to the /path topic and will pass it to the YarpFeetDataProcessor object
class SetpointConverter : public rclcpp::Node
{
private:
    const std::string m_topic_name = "/plan";     //subscribe to global plan topic
    yarp::os::Port m_feet_state_port;
    const std::string m_feet_state_port_name = "/setopint_converter/feet_state:i";
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer_in;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr m_setpoint_sub;
    YarpFeetDataProcessor m_processor;

    //Each time I have a new path -i.e. a new path is published - I pass it and set the flag to publish it as soon as the requirements are met.
   void msg_callback(const nav_msgs::msg::Path::ConstPtr& msg_in)
    {
        m_processor.storePath(msg_in);
        m_processor.set_permission(true);
    }
public:
    SetpointConverter() : rclcpp::Node("setpoint_converter_node")
    {   
        m_setpoint_sub = this->create_subscription<nav_msgs::msg::Path>(
            m_topic_name,
            10,
            std::bind(&SetpointConverter::msg_callback, this, _1)
        );

        m_tf_buffer_in = std::make_shared<tf2_ros::Buffer>(this->get_clock());    //share to the yarp port
        m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_in);

        //Create YarpFeetDataProcessor object
        m_processor.init(m_tf_buffer_in);
        m_feet_state_port.open(m_feet_state_port_name);
        yarp::os::Network::connect("/base-estimator/contacts/stateAndNormalForce:o", m_feet_state_port_name);
        m_feet_state_port.setReader(m_processor);
    }
};  //End of class SetpointConverter


int main(int argc, char** argv)
{
    // Init ROS2
    rclcpp::init(argc, argv);
    //Init YARP
    yarp::os::Network yarp;

    // Start listening in polling
    if (rclcpp::ok())
    {
        auto node = std::make_shared<SetpointConverter>();
        rclcpp::spin(node);
    }
    std::cout << "Shutting down" << std::endl;
    rclcpp::shutdown();
    return 0;
}