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



using namespace std::chrono_literals;
using std::placeholders::_1;

//Class used for YARP port callbacks. Monitors the feet contacts state.
class YarpDataProcessor : public yarp::os::PortReader
{
private:
    /* data */
    
public:
    YarpDataProcessor()
    {
        
    };

    bool read(yarp::os::ConnectionReader& connection) override
    {
        yarp::os::Bottle b;
        bool ok = b.read(connection);
        if (!ok) {
            std::cout << "Bad Yarp connection \n";
            return false;
        }
        
        
        return true;
    }
};  //End of class YarpFeetDataProcessor : public yarp::os::PortReader


//This class subscribes to the /path topic and will pass it to the YarpFeetDataProcessor object
class VirtualUnicyclePub : public rclcpp::Node
{
private:
    yarp::os::Port m_feet_state_port;
    const std::string m_feet_state_port_name = "/virtual_unicycle_publisher/unicycle_state:i";
    YarpDataProcessor m_processor;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer_in;

public:
    VirtualUnicyclePub() : rclcpp::Node("virtual_unicycle_publisher_node")
    {   
        //create TF
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        m_tf_buffer_in = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_in);
        //Create YarpDataProcessor object
        m_feet_state_port.open(m_feet_state_port_name);
        yarp::os::Network::connect("/walking-coordinator/virtual_unicycle_states:o", m_feet_state_port_name);   //base-estimator/contacts/stateAndNormalForce:o
        m_feet_state_port.setReader(m_processor);
    }

    bool publish(yarp::os::Bottle& data)
    {
        geometry_msgs::msg::TransformStamped tf, tfReference;
        tf.header.frame_id = "virtual_unicycle_simulated";
        tfReference.header.frame_id = "virtual_unicycle_reference";
        tf.header.stamp = now();
        tfReference.header.stamp = tf.header.stamp;
        if (data.get(2).asString() == "left")
        {
            tf.child_frame_id = "l_sole";
            tfReference.child_frame_id = "l_sole";
        }
        else
        {
            tf.child_frame_id = "r_sole"; 
            tfReference.child_frame_id = "r_sole";           
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

        m_tf_broadcaster->sendTransform(tf);
        m_tf_broadcaster->sendTransform(tfReference);
        
    }
};  //End of class VirtualUnicyclePub


int main(int argc, char** argv)
{
    // Init ROS2
    rclcpp::init(argc, argv);
    //Init YARP
    yarp::os::Network yarp;

    // Start listening in polling
    if (rclcpp::ok())
    {
        auto node = std::make_shared<VirtualUnicyclePub>();
        rclcpp::spin(node);
    }
    std::cout << "Shutting down" << std::endl;
    rclcpp::shutdown();
    return 0;
}