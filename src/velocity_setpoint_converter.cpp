#include "yarp/os/Bottle.h"
#include "yarp/os/Port.h"
#include "yarp/os/BufferedPort.h"
#include "yarp/os/Network.h"
#include "yarp/os/RpcClient.h"
#include "yarp/sig/Vector.h"
#include "yarp/os/ConnectionWriter.h"
#include "yarp/os/Portable.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <list>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;

//Class used for YARP port callbacks. Monitors the feet contacts state.
class YarpFeetDataProcessor : public yarp::os::PortReader
{
private:
    /* data */
    geometry_msgs::msg::Twist m_cmd_vel;
    //yarp::os::Port m_port;
    yarp::os::BufferedPort<yarp::sig::VectorOf<double> > m_port;
    const double m_sensor_threshold = 100.0;
    const std::string m_port_name = "/velocity_setopint_converter/talker:o";
    const std::string m_server_name = "/walking-coordinator/goal:i";
    bool m_send_goal;
    std::vector<double> m_theta_buffer;  //five dimensional buffer
public:
    YarpFeetDataProcessor()
    {
        m_send_goal = false;
        //Connection to the walking-controller goal port
        m_port.open(m_port_name);
        yarp::os::Network::connect(m_port_name, m_server_name);     //todo - check for connection or errors
    };

    void storeSetpoint(const geometry_msgs::msg::Twist::ConstPtr& msg)
    {
        //std::vector<double>::iterator it;
        //it = m_theta_buffer.begin();
        //m_theta_buffer.insert(it, msg->angular.z);
        //if (m_theta_buffer.size()>5)
        //{
        //    m_theta_buffer.pop_back();
        //}
        //// Compute the average theta speed
        //double average_theta_speed = 0.0;
        //for (auto itr : m_theta_buffer)
        //{
        //    average_theta_speed += itr;
        //}
        //average_theta_speed /= m_theta_buffer.size();
        
        m_cmd_vel.linear = msg->linear;
        //m_cmd_vel.angular.x = .0;
        //m_cmd_vel.angular.y = .0;
        //m_cmd_vel.angular.z = average_theta_speed;
        //m_cmd_vel->angular.z = average_theta_speed;
        m_cmd_vel.angular=msg->angular; //for debug ignore buffer
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
                std::vector<double> msg_out {m_cmd_vel.linear.x, m_cmd_vel.angular.z, m_cmd_vel.linear.y};
                auto& out = m_port.prepare();
                out.clear();
                out.push_back(m_cmd_vel.linear.x);
                out.push_back(m_cmd_vel.angular.z);
                out.push_back(m_cmd_vel.linear.y);
                std::cout << "Cmd_Vel V_x: " << m_cmd_vel.linear.x << " V_theta: " << m_cmd_vel.angular.z << " V_y: " << m_cmd_vel.linear.y << std::endl;
                std::cout << "Passing Setpoint V_x: " << out[0] << " V_theta: " << out[1] << " V_y: " << out[2] << std::endl;
                m_port.write();  //send data only once per double support
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
    const std::string m_topic_name = "/cmd_vel";     //subscribe to cmd_vel
    yarp::os::Port m_feet_state_port;
    const std::string m_feet_state_port_name = "/setopint_converter/feet_state:i";
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_setpoint_sub;
    YarpFeetDataProcessor m_processor;

    //Each time I have a new path -i.e. a new path is published - I pass it and set the flag to publish it as soon as the requirements are met.
   void msg_callback(const geometry_msgs::msg::Twist::ConstPtr& msg_in)
    {
        m_processor.storeSetpoint(msg_in);
        m_processor.set_permission(true);
    }
public:
    SetpointConverter() : rclcpp::Node("setpoint_converter_node")
    {   
        m_setpoint_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            m_topic_name,
            10,
            std::bind(&SetpointConverter::msg_callback, this, _1)
        );

        //Create YarpFeetDataProcessor object
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