#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <yarp/os/Bottle.h>
#include <yarp/os/Port.h>
#include <yarp/os/Network.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Time.h>

#include <chrono>
#include <thread>
#include <memory>
#include <string>

using namespace std::chrono_literals;
using yarp::os::Bottle;

class ManagedOdom : public rclcpp_lifecycle::LifecycleNode
{
public:
    // Constructor
    explicit ManagedOdom(const std::string & node_name, bool use_IPC = false)
    : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(use_IPC))
    {}

    // Sends the odom msg and publishes the odom frame
    void publish()
    {
        try
        {
            if (!odom_pub_->is_activated())
            {
                RCLCPP_INFO(get_logger(), "Lifecycle publisher is currently inactive. Messages are not published.\n");
            }
            else
            {
                RCLCPP_INFO(get_logger(), "Publishing...\n");
                tf_broadcaster_->sendTransform(odom_tf);
                odom_pub_->publish(odom_msg);
            }
        }
        catch(const std::exception& e)
        {
            RCLCPP_WARN(this->get_logger(), "Unable to publish odom message and frame: %s \n",e.what());
        }
    }

    // Gets the latest TF available
    bool get_TF(const char* target_link)
    {
        while (rclcpp::ok())
        {
            try
            {
                tf_in = tfBuffer_in_->lookupTransform(target_link, root_link_name, rclcpp::Time(0), 300ms); // target link = chest
                return true;
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "%s \n", ex.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                //continue;
            }
        }
        return false;
    }

    // Reads IMU and Estimator YARP ports, computes the TFs and composes the odom and tf msgs
    bool compute_odom()
    {
        try
        {
            RCLCPP_INFO(get_logger(), "compute_odom() start\n");
            Bottle in_bottle;
            odom_reader_port.read(in_bottle);
            Bottle imu_bottle;
            imu_reader_port.read(imu_bottle);
            if (! xy_offsets_computed)
            {
                initial_offset_x = in_bottle.get(0).asFloat64();
                initial_offset_y = in_bottle.get(1).asFloat64();
                xy_offsets_computed = true;
            }
            if (! get_TF("chest"))  // updates with the most recent tf
            {
                throw std::logic_error("Cannot get TF ");
            }

            // TF Translation
            odom_tf.transform.translation.x = in_bottle.get(0).asFloat64() - initial_offset_x - tf_in.transform.translation.x;
            odom_tf.transform.translation.y = in_bottle.get(1).asFloat64() - initial_offset_y - tf_in.transform.translation.y;
            odom_tf.transform.translation.z = in_bottle.get(2).asFloat64() + tf_in.transform.translation.z;
            // TF Rotation
            tf2::Quaternion msg_quat;    //converted quat received on the get_TF method
            tf2::fromMsg(tf_in.transform.rotation, msg_quat);
            tf2::Quaternion imu_q;
            imu_q.setRPY(0, 0, imu_bottle.get(3).asList()->get(0).asList()->get(0).asList()->get(2).asFloat64()* 0.0174533);
            tf2::Quaternion new_quat;   //imu quat in root_link frame
            new_quat = imu_q * msg_quat;

            // odom TF
            odom_tf.transform.rotation.x = new_quat.x();
            odom_tf.transform.rotation.y = new_quat.y();
            odom_tf.transform.rotation.z = new_quat.z();
            odom_tf.transform.rotation.w = new_quat.w();

            // odom msg
            odom_msg.pose.pose.orientation = odom_tf.transform.rotation;
            odom_msg.pose.pose.position.x = odom_tf.transform.translation.x;
            odom_msg.pose.pose.position.y = odom_tf.transform.translation.y;
            odom_msg.pose.pose.position.z = odom_tf.transform.translation.z;
            odom_msg.twist.twist.linear.x = in_bottle.get(6).asFloat64();
            odom_msg.twist.twist.linear.y = in_bottle.get(7).asFloat64();
            odom_msg.twist.twist.linear.z = in_bottle.get(8).asFloat64();
            odom_msg.twist.twist.angular.x = in_bottle.get(9).asFloat64();
            odom_msg.twist.twist.angular.y = in_bottle.get(10).asFloat64();
            odom_msg.twist.twist.angular.z = in_bottle.get(11).asFloat64();
            // time stamps
            odom_msg.header.stamp = rclcpp::Time(yarp::os::Time::now());
            odom_tf.header.stamp = odom_msg.header.stamp;
            RCLCPP_INFO(get_logger(), "compute_odom() end\n");
            return true;
        }
        catch(const std::exception& e)
        {
            RCLCPP_WARN(this->get_logger(), "%s \n",e.what());
            return false;
        }
        return true;
    }

    // Manages the logic flow for each clock tick
    void on_timer()
    {
        if (this->compute_odom())
        {
            this->publish();
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Unable to COMPUTE odom message and frame\n");
        }
    }

    // ON CONFIGURE
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("estimated_odom", 10);
        //timer_ = this->create_wall_timer(20ms, std::bind(&ManagedOdom::on_timer, this));    //todo parameterize loop period // Should be on activate
        RCLCPP_INFO(get_logger(), "on_configure() is called.\n");
        // Create YARP ports
        odom_reader_port.open(reader_port_name);
        imu_reader_port.open(imu_reader_port_name);
        //Connect YARP ports
        RCLCPP_INFO(get_logger(), "Connecting YARP ports...\n");
        yarp::os::Network::connect(writer_port, reader_port_name);
        yarp::os::Network::connect(imu_writer_port, imu_reader_port_name);
        // tf2
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tfBuffer_in_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_in_);
        // Constant parameters setup
        odom_msg.header.frame_id = odom_frame_name;
        odom_msg.child_frame_id = root_link_name;
        /* set up odom frame */
        odom_tf.child_frame_id = root_link_name;
        odom_tf.header.frame_id = odom_frame_name;
        // TODO - Tune covariance matricies
        odom_msg.pose.covariance = {0.05, 0    , 0   , 0   , 0   , 0   , 
                                    0   , 0.05 , 0   , 0   , 0   , 0   , 
                                    0   , 0    , 0.02, 0   , 0   , 0   , 
                                    0   , 0    , 0   , 0.06, 0   , 0   ,
                                    0   , 0    , 0   , 0   , 0.06, 0   , 
                                    0   , 0    , 0   , 0   , 0   , 0.06};
        odom_msg.twist.covariance = {0.1, 0    , 0   , 0   , 0   , 0   , 
                                    0   , 0.1  , 0   , 0   , 0   , 0   , 
                                    0   , 0    , 0.05, 0   , 0   , 0   , 
                                    0   , 0    , 0   , 0.12, 0   , 0   ,
                                    0   , 0    , 0   , 0   , 0.12, 0   , 
                                    0   , 0    , 0   , 0   , 0   , 0.12};
        RCLCPP_INFO(get_logger(), "Finished configuring\n");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    //ON ACTIVATE
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & state)
    {
        RCLCPP_INFO(get_logger(), "on_activate() is called.");
        timer_ = this->create_wall_timer(20ms, std::bind(&ManagedOdom::on_timer, this));    //todo parameterize loop period
        LifecycleNode::on_activate(state);
        odom_pub_->on_activate();
        //
        initial_offset_x = 0;
        initial_offset_y = 0;
        xy_offsets_computed = false;
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // ON DEACTIVATE
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & state)
    {
        LifecycleNode::on_deactivate(state);
        odom_pub_->on_deactivate();
        RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.\n");
        // Disconnect YARP ports
        RCLCPP_INFO(get_logger(), "Disconnecting YARP ports...\n");
        yarp::os::Network::disconnect(writer_port, reader_port_name);
        yarp::os::Network::disconnect(imu_writer_port, imu_reader_port_name);
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    //ON CLEANUP
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &)
    {
        // Destroy smart ptrs
        timer_.reset();
        odom_pub_.reset();
        tf_broadcaster_.reset();
        tfBuffer_in_.reset();
        tf_listener_.reset();
        RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");
        // Close YARP ports
        odom_reader_port.close();
        imu_reader_port.close();
        
        initial_offset_x = 0;
        initial_offset_y = 0;
        xy_offsets_computed = false;
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    //ON SHUTDOWN
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State & state)
    {
        timer_.reset();    
        odom_pub_.reset();
        tf_broadcaster_.reset();
        tfBuffer_in_.reset();
        tf_listener_.reset();
        RCUTILS_LOG_INFO_NAMED( get_name(),
            "on shutdown is called from state %s.",
            state.label().c_str());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    
    //ON ERROR
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_error(const rclcpp_lifecycle::State &)
    {
        // Todo - error management
        RCUTILS_LOG_INFO_NAMED(get_name(), "something went wrong!");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

private:
    // Lifecycle management
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>> odom_pub_;
    std::shared_ptr<rclcpp::TimerBase> timer_;  // upon creation
    
    //Odom output msgs
    nav_msgs::msg::Odometry odom_msg;
    geometry_msgs::msg::TransformStamped odom_tf;
    //Input msgs
    geometry_msgs::msg::TransformStamped tf_in;

    //YARP PORTS
    yarp::os::Port odom_reader_port;
    yarp::os::Port imu_reader_port;
    const char* reader_port_name = "/managed_odom/odom_reader";
    const char* writer_port = "/base-estimator/floating_base/state:o";
    const char* imu_reader_port_name = "/managed_odom/imu_reader";
    const char* imu_writer_port = "/icubSim/chest/inertials/measures:o";

    // TF2 ROS2
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tfBuffer_in_;

    // Constants
    const char* odom_topic = "estimated_odom";
    const char* chest_imu_topic = "chest_imu";
    const char* odom_frame_name = "odom";
    const char* root_link_name = "root_link";

    // Offsets
    double initial_offset_x, initial_offset_y;
    bool xy_offsets_computed;
};

int main(int argc, char ** argv)
{
    // force flush of the stdout buffer.
    // this ensures a correct sync of all prints
    // even when executed simultaneously within the launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    // Network init
    yarp::os::Network yarp;
    rclcpp::init(argc, argv);
    // Node run
    rclcpp::executors::SingleThreadedExecutor exe;
    std::shared_ptr<ManagedOdom> lc_node = std::make_shared<ManagedOdom>("managed_odom_node");
    exe.add_node(lc_node->get_node_base_interface());
    exe.spin();

    rclcpp::shutdown();
    return 0;
}