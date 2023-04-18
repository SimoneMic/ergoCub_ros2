#include "yarp/os/Bottle.h"
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
#include "visualization_msgs/msg/marker_array.hpp"

#include <mutex>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

//This class has the purpose to publish data on ros topics
class FootstepsViewerRos : public rclcpp::Node
{
private:

    //std::shared_ptr<tf2_ros::TransformListener> m_tfListener{nullptr};
    //std::shared_ptr<tf2_ros::Buffer> m_tfBuffer;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_rightFootprintsMarkersPub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_leftFootprintsMarkersPub;
    const std::string m_rightFootprintsTopicName = "/footstep_viewer_node/right_footprints";
    const std::string m_leftFootprintsTopicName = "/footstep_viewer_node/left_footprints";

    std::mutex m_rosMutex;
    
public:
    FootstepsViewerRos() : rclcpp::Node("footstep_viewer_node")
    {   
        //m_tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        //m_tfListener = std::make_shared<tf2_ros::TransformListener>(*m_tfBuffer);
        m_rightFootprintsMarkersPub = this->create_publisher<visualization_msgs::msg::MarkerArray>(m_rightFootprintsTopicName, 10);
        m_leftFootprintsMarkersPub = this->create_publisher<visualization_msgs::msg::MarkerArray>(m_leftFootprintsTopicName, 10);
    }

    bool publishMarkers(const yarp::os::Bottle& data){
        //std::lock_guard<std::mutex> guard(m_rosMutex);
        std::cout << "publishMarkers" << std::endl;
        std::cout << data.size() << std::endl;
        auto leftSteps = data.get(0).asList();
        auto rightSteps = data.get(1).asList();
        if (leftSteps->size() == 0 || rightSteps->size()==0)
        {
            RCLCPP_INFO(this->get_logger(), "One of the Step array is empty");
            return false;
        }

        visualization_msgs::msg::MarkerArray right_marker_array;
        visualization_msgs::msg::MarkerArray left_marker_array;
        //Clear the previous markers
        visualization_msgs::msg::Marker clear_msg;
        clear_msg.id = 0;
        clear_msg.ns = "my_namespace";
        clear_msg.action = visualization_msgs::msg::Marker::DELETEALL;
        
        right_marker_array.markers.push_back(clear_msg);
        left_marker_array.markers.push_back(clear_msg);
        m_leftFootprintsMarkersPub->publish(left_marker_array);
        m_rightFootprintsMarkersPub->publish(right_marker_array);

        left_marker_array.markers.clear();
        right_marker_array.markers.clear();
        builtin_interfaces::msg::Time timestamp = now();
        //LEFT
        std::cout << "Left Loop" << std::endl;
        //RCLCPP_INFO(this->get_logger(), "Left Loop");
        for (size_t i = 0; i < leftSteps->size(); ++i)
        {
            visualization_msgs::msg::Marker tmp_marker_msg;
            tmp_marker_msg.header.frame_id = "odom";
            tmp_marker_msg.id = i;
            tmp_marker_msg.ns = "my_namespace";
            tmp_marker_msg.header.stamp = timestamp;
            tmp_marker_msg.scale.x = 0.05;
            tmp_marker_msg.scale.y = 0.05;
            tmp_marker_msg.scale.z = 0.05;
            // Color for left foot
            tmp_marker_msg.color.r = 0.0;
            tmp_marker_msg.color.g = 1.0;
            tmp_marker_msg.color.b = 0.0;
            tmp_marker_msg.color.a = 1.0;
            tmp_marker_msg.type = visualization_msgs::msg::Marker::ARROW;
            tmp_marker_msg.pose.position.x = leftSteps->get(i).asList()->get(0).asFloat64();
            tmp_marker_msg.pose.position.y = leftSteps->get(i).asList()->get(1).asFloat64();
            tmp_marker_msg.pose.position.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, leftSteps->get(i).asList()->get(2).asFloat64());
            tmp_marker_msg.pose.orientation = tf2::toMsg(q);
            tmp_marker_msg.frame_locked = true;
            tmp_marker_msg.action = visualization_msgs::msg::Marker::ADD;
            //Populate the marker with atleast one mesh point
            geometry_msgs::msg::Point cube_center;
            cube_center.x = 0.0;
            cube_center.y = 0.0;
            cube_center.z = 0.0;
            tmp_marker_msg.points.push_back(cube_center);
            cube_center.x = 0.1;
            cube_center.y = 0.0;
            cube_center.z = 0.0;
            tmp_marker_msg.points.push_back(cube_center);
            //save marker in the array
            left_marker_array.markers.push_back(tmp_marker_msg);
        }
        //RCLCPP_INFO(this->get_logger(), "Publishing Left");
        std::cout << "Left Publish" << std::endl;
        m_leftFootprintsMarkersPub->publish(left_marker_array);

        //RIGHT
        std::cout << "Right Loop" << std::endl;
        //RCLCPP_INFO(this->get_logger(), "Right Loop");

        //tmp_marker_msg.points.clear();
        for (size_t i = 0; i < rightSteps->size(); ++i)
        {
            visualization_msgs::msg::Marker tmp_marker_msg;
            tmp_marker_msg.header.frame_id = "odom";
            tmp_marker_msg.id = i;
            tmp_marker_msg.ns = "my_namespace";
            tmp_marker_msg.header.stamp = timestamp;
            tmp_marker_msg.scale.x = 0.05;
            tmp_marker_msg.scale.y = 0.05;
            tmp_marker_msg.scale.z = 0.05;
            // Color for left foot
            tmp_marker_msg.color.r = 1.0;
            tmp_marker_msg.color.g = 0.0;
            tmp_marker_msg.color.b = 0.0;
            tmp_marker_msg.color.a = 1.0;
            tmp_marker_msg.type = visualization_msgs::msg::Marker::ARROW;
            tmp_marker_msg.pose.position.x = rightSteps->get(i).asList()->get(0).asFloat64();
            tmp_marker_msg.pose.position.y = rightSteps->get(i).asList()->get(1).asFloat64();
            tmp_marker_msg.pose.position.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, rightSteps->get(i).asList()->get(2).asFloat64());
            tmp_marker_msg.pose.orientation = tf2::toMsg(q);
            tmp_marker_msg.frame_locked = true;
            tmp_marker_msg.action = visualization_msgs::msg::Marker::ADD;
            //Populate the marker with atleast 2 mesh point for ARROW
            geometry_msgs::msg::Point cube_center;
            cube_center.x = 0.0;
            cube_center.y = 0.0;
            cube_center.z = 0.0;
            tmp_marker_msg.points.push_back(cube_center);
            cube_center.x = 0.1;
            cube_center.y = 0.0;
            cube_center.z = 0.0;
            tmp_marker_msg.points.push_back(cube_center);
            //save marker in the array
            right_marker_array.markers.push_back(tmp_marker_msg);
        }

        //RCLCPP_INFO(this->get_logger(), "Publishing Right");
        std::cout << "Right Publish" << std::endl;
        //publish
        m_rightFootprintsMarkersPub->publish(right_marker_array);
        std::cout << "Exiting" << std::endl;


    }
};  //End of class FootstepsViewerRos

//Class used for YARP port callbacks. Monitors the feet contacts state.
class FootstepsViewerYarp : public yarp::os::PortReader
{
private:
    std::mutex m_mutex;
    std::shared_ptr<FootstepsViewerRos> m_rosNode;
    bool m_initialized = false;

public:

    FootstepsViewerYarp()
    {
    };
    //main loop executed for each port reading of the merged feet status
    bool read(yarp::os::ConnectionReader& t_connection) override
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        if (!m_initialized)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        std::cout << "New message received" << std::endl;
        yarp::os::Bottle b;
        bool ok = b.read(t_connection);
        if (!ok) {
            std::cout << "No connection available for reading data " << std::endl;
            return false;
        }
        if (m_initialized)
        {
            try
            {
                m_rosNode->publishMarkers(b);
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
        }
        else
        {
            std::cout << "ROS node not yet initialized" << std::endl;
        }
        return true;
    }

    void runROS()
    {
        m_rosNode = std::make_shared<FootstepsViewerRos>();
        m_initialized = true;
        rclcpp::spin(m_rosNode);
    }


    
};  //End of class FootstepsViewerYarp : public yarp::os::PortReader

int main(int argc, char** argv)
{
    const std::string footprintsPortName = "/footsteps_viewer/footsteps:i";
    // Init ROS2
    rclcpp::init(argc, argv);
    //Init YARP
    yarp::os::Network yarp;
    // Start listening in polling
    if (rclcpp::ok())
    {
        FootstepsViewerYarp dataProcessor;
        yarp::os::Port footprintsPort;
        footprintsPort.open(footprintsPortName);
        yarp::os::Network::connect("/navigation_helper/feet_positions:o", footprintsPortName);
        footprintsPort.setReader(dataProcessor);
        dataProcessor.runROS();
        //rclcpp::spin(node);
    }
    std::cout << "Shutting down" << std::endl;
    rclcpp::shutdown();
    return 0;
}