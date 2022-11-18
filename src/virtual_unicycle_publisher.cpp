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
            std::cout << "Reading Transform: X: " <<  data.get(3).asList()->get(0).asFloat64() << " Y: " <<  data.get(3).asList()->get(1).asFloat64() <<
                         " Z: " << data.get(3).asList()->get(2).asFloat64() << " x: " << data.get(3).asList()->get(3).asFloat64() <<
                         " y: " << data.get(3).asList()->get(4).asFloat64() << " z: " << data.get(3).asList()->get(5).asFloat64() <<
                          std::endl;
            std::cout << "Time: " << now().seconds() << " " << now().nanoseconds() << std::endl;
            geometry_msgs::msg::TransformStamped tf, tfReference;
            tf.header.stamp = now();
            tfReference.header.stamp = tf.header.stamp;
            geometry_msgs::msg::TransformStamped tf_fromOdom, tfReference_fromOdom; //position of the virtual unicycle computed from the walking-controller in the odom frame
            tf_fromOdom.header.stamp = tfReference_fromOdom.header.stamp = tf.header.stamp;
            tf_fromOdom.child_frame_id = "odom_virtual_unicycle_simulated";
            tfReference_fromOdom.child_frame_id = "odom_virtual_unicycle_reference";
            tf_fromOdom.header.frame_id = "odom";
            tfReference_fromOdom.header.frame_id = "odom";


            if (data.get(2).asString() == "left")
            {
                tf.header.frame_id = "l_sole";
                tfReference.header.frame_id = "l_sole";
                tf.child_frame_id = "virtual_unicycle_simulated";
                tfReference.child_frame_id = "virtual_unicycle_reference";
                tf.transform.translation.y = - 0.07;
            }
            else
            {
                tf.header.frame_id = "r_sole";
                tfReference.header.frame_id = "r_sole";
                tf.child_frame_id = "virtual_unicycle_simulated"; 
                tfReference.child_frame_id = "virtual_unicycle_reference";   
                tf.transform.translation.y = 0.07;        
            }

            tf.transform.translation.x = 0;
            tf.transform.translation.z = 0;
            tfReference.transform.translation.x = tf.transform.translation.x + 0.1;
            tfReference.transform.translation.y = tf.transform.translation.y;
            tfReference.transform.translation.z = 0.0;
            //Odom Computation
            geometry_msgs::msg::TransformStamped odomTf;
            odomTf.header.frame_id = "odom";
            odomTf.child_frame_id = "root_link";
            odomTf.header.stamp = tf.header.stamp;
            odomTf.transform.translation.x = data.get(3).asList()->get(0).asFloat64();
            odomTf.transform.translation.y = data.get(3).asList()->get(1).asFloat64();
            odomTf.transform.translation.z = data.get(3).asList()->get(2).asFloat64();

            tf2::Quaternion qOdom;
            qOdom.setRPY(data.get(3).asList()->get(3).asFloat64(), data.get(3).asList()->get(4).asFloat64(), data.get(3).asList()->get(5).asFloat64());
            odomTf.transform.rotation.x = qOdom.x();
            odomTf.transform.rotation.y = qOdom.y();
            odomTf.transform.rotation.z = qOdom.z();
            odomTf.transform.rotation.w = qOdom.w();

            m_tf_broadcaster->sendTransform(odomTf);

            //Virtual unicycle base pub in odom frame
            tf_fromOdom.transform.translation.x = data.get(0).asList()->get(0).asFloat64();
            tf_fromOdom.transform.translation.y = data.get(0).asList()->get(1).asFloat64();
            tf_fromOdom.transform.translation.z = 0.0;

            tfReference_fromOdom.transform.translation.x = data.get(1).asList()->get(0).asFloat64();
            tfReference_fromOdom.transform.translation.y = data.get(1).asList()->get(1).asFloat64();
            tfReference_fromOdom.transform.translation.z = 0.0;

            tf2::Quaternion qVirtualUnicycleInOdomFrame;
            qVirtualUnicycleInOdomFrame.setRPY(0, 0, data.get(0).asList()->get(2).asFloat64());
            tf_fromOdom.transform.rotation.x = qVirtualUnicycleInOdomFrame.x();
            tf_fromOdom.transform.rotation.y = qVirtualUnicycleInOdomFrame.y();
            tf_fromOdom.transform.rotation.z = qVirtualUnicycleInOdomFrame.z();
            tf_fromOdom.transform.rotation.w = qVirtualUnicycleInOdomFrame.w();
            tfReference_fromOdom.transform.rotation = tf_fromOdom.transform.rotation;

            geometry_msgs::msg::TransformStamped footToRootTF;
            if (data.get(2).asString() == "left")
            {
                footToRootTF = m_tf_buffer_in->lookupTransform("root_link", "l_sole", rclcpp::Time(0));
            }
            else
            {
                footToRootTF = m_tf_buffer_in->lookupTransform("root_link", "r_sole", rclcpp::Time(0));
            }
            //Conversion for extracting only YAW
            tf2::Quaternion tfGround;    //quat of the root_link to chest frame tf
            tf2::fromMsg(footToRootTF.transform.rotation, tfGround);
            //Conversion from quat to rpy -> possible computational errors due to matricies
            tf2::Matrix3x3 m(tfGround);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            tf.transform.rotation.x = q.x();
            tf.transform.rotation.y = q.y();
            tf.transform.rotation.z = q.z();
            tf.transform.rotation.w = q.w();

            tf2::Quaternion qRef;
            qRef.setRPY(0, 0, yaw);
            tfReference.transform.rotation.x = qRef.x();
            tfReference.transform.rotation.y = qRef.y();
            tfReference.transform.rotation.z = qRef.z();
            tfReference.transform.rotation.w = qRef.w();
            std::cout << "Publishing tf X: " <<  tf.transform.translation.x << " Y: " <<  tf.transform.translation.y << std::endl;

            m_tf_broadcaster->sendTransform(tf);
            m_tf_broadcaster->sendTransform(tfReference);
            m_tf_broadcaster->sendTransform(tf_fromOdom);
            m_tf_broadcaster->sendTransform(tfReference_fromOdom);

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