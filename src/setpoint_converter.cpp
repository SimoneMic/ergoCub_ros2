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

//Class used for YARP port callbacks
class YarpFeetDataProcessor : public yarp::os::PortReader
{
private:
    /* data */
    nav_msgs::msg::Path::ConstPtr path_msg;
    //bool on_double_support;     //flag for when the robot enters on double support
    yarp::os::Port port;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    const double sensor_threshold = 100.0;

    const std::string port_name = "/setopint_converter/talker:o";
    const std::string server_name = "/walking-coordinator/goal:i";

    bool send_goal;
public:
    YarpFeetDataProcessor()
    {
        //on_double_support = false;
        send_goal = false;
        port.open(port_name);
        yarp::os::Network::connect(port_name, server_name);     //todo - check for connection or errors
    };

    void init(std::shared_ptr<tf2_ros::Buffer> &buffer)
    {
        tf_buffer_ = buffer;
    }

    void storePath(const nav_msgs::msg::Path::ConstPtr& msg)
    {
        path_msg = msg;
    }

    //set the internal flag to whether send the goal or not
    void set_permission(bool val)
    {
        send_goal = val;
    }

    bool read(yarp::os::ConnectionReader& connection) override
    {
        yarp::os::Bottle b;
        bool ok = b.read(connection);
        if (!ok) {
            std::cout << "Bad Yarp connection \n";
            return false;
        }
        //Condition for double support -> Should also check if in the middle of the CoM path during oscillation???
        if (b.get(0).asFloat64() > sensor_threshold && b.get(1).asFloat64() > sensor_threshold)
        {
            //check for data sanity before transmitting it
            //todo
            if (send_goal)  //This means that I have a new path
            {
                //I have to transform the goal to the robot reference frame (root_link) only when I have to send the data
                yarp::os::Bottle msg_out;
                geometry_msgs::msg::PoseStamped goal_robot;
                goal_robot.pose.position.x = 0;
                goal_robot.pose.position.y = 0;
                try
                {
                    //
                    geometry_msgs::msg::TransformStamped  TF = tf_buffer_->lookupTransform("root_link", path_msg->header.frame_id, rclcpp::Time(0), 100ms);
                    TF.transform.translation.x += 0.1;  //offsetted reference point used by the walking-controller -> found in config file by person distance
                    //todo: implement a slicing of the path
                    tf2::doTransform(path_msg->poses.back(), goal_robot, TF);
                }
                catch(const std::exception& e)
                {
                    std::cerr << "Transform Exception: " << e.what() << "\n";
                    goal_robot.pose.position.x = 0;
                    goal_robot.pose.position.y = 0;
                }
                msg_out.addFloat64(goal_robot.pose.position.x);
                msg_out.addFloat64(goal_robot.pose.position.y);
                std::cout << "Passing Setpoint x: " << msg_out.get(0).asFloat64() << " y: " << msg_out.get(1).asFloat64() << "\n";
                port.write(msg_out);  //send data only once per double support
                send_goal = false;
            }
        }
        return true;
    }
};


class SetpointConverter : public rclcpp::Node
{
private:
    const std::string topic_name = "/plan";     //subscribe to global plan
    //const char* port_name = "/setopint_converter/talker:o";
    //const char* server_name = "/walking-coordinator/goal:i";
    //yarp::os::Port port;
    yarp::os::Port feet_state_port;
    const std::string feet_state_port_name = "/setopint_converter/feet_state:i";
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_in;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr setpoint_sub_;
    YarpFeetDataProcessor processor;

   void msg_callback(const nav_msgs::msg::Path::ConstPtr& msg_in)
    {
        /*
        yarp::os::Bottle msg_out;
        // Need to transform the path from map frame to robot frame
        geometry_msgs::msg::PoseStamped goal_robot;
        //tf_buffer_in->transform(msg_in->poses.back(), goal_robot, "chest", 200ms);    //goal pose in robot frame: could throw exception for extrapolation into the future
        try
        {
            //
            geometry_msgs::msg::TransformStamped  TF = tf_buffer_in->lookupTransform("root_link", msg_in->header.frame_id, rclcpp::Time(0), 100ms);
            TF.transform.translation.x += 0.1;  //offsetted reference point used by the walking-controller
            //todo: implement a slicing of the path
            tf2::doTransform(msg_in->poses.back(), goal_robot, TF);
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform Exception: %s \n", e.what());
        }
        msg_out.addFloat64(goal_robot.pose.position.x);
        msg_out.addFloat64(goal_robot.pose.position.y);
        RCLCPP_INFO(this->get_logger(), "STORING Setpoint x: %f y: %f \n", msg_out.get(0).asFloat64(), msg_out.get(1).asFloat64());
        //I should publish the command only when the robot is in double support -> other callback
        //port.write(msg_out);
        */
        processor.storePath(msg_in);
        processor.set_permission(true);
    }
public:
    SetpointConverter() : rclcpp::Node("setpoint_converter_node")
    {
        // Network Setup
        //yarp::os::Network yarp;
        
        setpoint_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            topic_name,
            10,
            std::bind(&SetpointConverter::msg_callback, this, _1)
        );

        tf_buffer_in = std::make_shared<tf2_ros::Buffer>(this->get_clock());    //share to the yarp port
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_in);

        //Create YarpFeetDataProcessor object
        processor.init(tf_buffer_in);
        feet_state_port.open(feet_state_port_name);
        yarp::os::Network::connect("/base-estimator/contacts/stateAndNormalForce:o", feet_state_port_name);
        feet_state_port.setReader(processor);
    }
};


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