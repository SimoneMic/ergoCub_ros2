#include "yarp/os/Bottle.h"
#include "yarp/os/Port.h"
#include "yarp/os/BufferedPort.h"
#include "yarp/os/Network.h"
#include "yarp/sig/Vector.h"
#include "yarp/os/ConnectionWriter.h"
#include "yarp/os/Portable.h"

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <mutex>
#include <thread>

using namespace std::chrono_literals;
using std::placeholders::_1;

class VirtualUnicyclePub : public rclcpp::Node
{
private:
    const std::string port_name = "/virtual_unicycle_publisher/unicycle_states:i";
    yarp::os::Port port;
    const double m_loopFreq = 100.0;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer_in;
    std::mutex m_mutex;
    rclcpp::TimerBase::SharedPtr m_timer;
public:
    VirtualUnicyclePub() : rclcpp::Node("virtual_unicycle_publisher_node")
    {   
        port.open(port_name);
        yarp::os::Network::connect("/walking-coordinator/virtual_unicycle_states:o", port_name); 
        //create TF
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        m_tf_buffer_in = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_in);
        auto duration = std::chrono::duration<double>(1/m_loopFreq);
        m_timer = this->create_wall_timer(duration, std::bind(&VirtualUnicyclePub::publish, this));
    }

    void publish()
    {
        //std::lock_guard<std::mutex> guard(m_mutex);
        try
        {
            yarp::os::Bottle data;
            port.read(data);
            std::cout << "publish" << std::endl;
            std::cout << "Reading virtual_unicycle_simulated: X: " <<  data.get(0).asList()->get(0).asFloat64() << " Y: " <<  data.get(0).asList()->get(1).asFloat64() <<
                         " Theta: " << data.get(0).asList()->get(2).asFloat64() << std::endl;
            std::cout << "Reading virtual_unicycle_reference: X: " <<  data.get(1).asList()->get(0).asFloat64() << " Y: " <<  data.get(1).asList()->get(1).asFloat64() <<
                         " Theta: " << data.get(1).asList()->get(2).asFloat64() << std::endl;
            std::cout << "Reading stance foot: " << data.get(2).asString() << std::endl;
            geometry_msgs::msg::TransformStamped tf, tfReference;
            tf.header.stamp = now();
            tfReference.header.stamp = tf.header.stamp;
            if (data.get(2).asString() == "left")
            {
                tf.header.frame_id = "l_sole";
                tfReference.header.frame_id = "l_sole";
                tf.child_frame_id = "virtual_unicycle_simulated";
                tfReference.child_frame_id = "virtual_unicycle_reference";
            }
            else
            {
                tf.header.frame_id = "r_sole";
                tfReference.header.frame_id = "r_sole";
                tf.child_frame_id = "virtual_unicycle_simulated"; 
                tfReference.child_frame_id = "virtual_unicycle_reference";           
            }

            tf.transform.translation.x = data.get(0).asList()->get(0).asFloat64();
            tf.transform.translation.y = data.get(0).asList()->get(1).asFloat64();
            tf.transform.translation.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0,0, data.get(0).asList()->get(2).asFloat64());
            tf.transform.rotation.x = q.x();
            tf.transform.rotation.y = q.y();
            tf.transform.rotation.z = q.z();
            tf.transform.rotation.w = q.w();

            tfReference.transform.translation.x = data.get(1).asList()->get(0).asFloat64();
            tfReference.transform.translation.y = data.get(1).asList()->get(1).asFloat64();
            tfReference.transform.translation.z = 0.0;

            tf2::Quaternion qRef;
            qRef.setRPY(0,0, data.get(1).asList()->get(2).asFloat64());
            tfReference.transform.rotation.x = qRef.x();
            tfReference.transform.rotation.y = qRef.y();
            tfReference.transform.rotation.z = qRef.z();
            tfReference.transform.rotation.w = qRef.w();
            std::cout << "Publishing tf X: " <<  tf.transform.translation.x << " Y: " <<  tf.transform.translation.y << std::endl;

            m_tf_broadcaster->sendTransform(tf);
            m_tf_broadcaster->sendTransform(tfReference);
            std::cout << "Exit publish" << std::endl;
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), e.what());
        }
    }
};  //End of class VirtualUnicyclePub

//Class used for YARP port callbacks. 
//class YarpDataProcessor : public yarp::os::PortReader
//{
//private:
//    /* data */
//    
//    std::mutex m_yarpMutex;
//public:
//    std::shared_ptr<VirtualUnicyclePub> m_ros;
//
//    YarpDataProcessor()
//    {
//        m_ros = std::make_shared<VirtualUnicyclePub>();
//    };
//
//    bool read(yarp::os::ConnectionReader& t_connection) override
//    {
//        std::lock_guard<std::mutex> guard(m_yarpMutex);
//        std::cout << "Got new message on port" << std::endl;
//        yarp::os::Bottle b;
//        bool ok = b.read(t_connection);
//        if (!ok) {
//            std::cout << "No connection available for reading data " << std::endl;
//            return false;
//        }
//        try
//        {
//            m_ros->publish(b);
//        }
//        catch(const std::exception& e)
//        {
//            std::cerr << e.what() << '\n';
//        }
//        
//    }
//};  //End of class YarpDataProcessor

int main(int argc, char** argv)
{
    // Init ROS2
    rclcpp::init(argc, argv);
    //Init YARP
    yarp::os::Network yarp;
    const std::string port_name = "/virtual_unicycle_publisher/unicycle_state:i";
    // Start listening in polling
    if (rclcpp::ok())
    {
        //YarpDataProcessor processor;
        //yarp::os::Port port;
        //port.open(port_name);
        //yarp::os::Network::connect("/walking-coordinator/virtual_unicycle_states:o", port_name); 
        //port.setReader(processor);
        auto node = std::make_shared<VirtualUnicyclePub>();
        std::cout << "Spinning..." << std::endl;
        rclcpp::spin(node);
    }
    std::cout << "Shutting down" << std::endl;
    rclcpp::shutdown();
    return 0;
}