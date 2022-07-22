#include "yarp/os/Bottle.h"
#include "yarp/os/BufferedPort.h"
#include "yarp/os/Port.h"
#include "yarp/os/Network.h"
#include "yarp/os/RpcClient.h"

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <odometry_callback.hpp>

//Class that computes odometry 
/*
Command to use to merge ports:
yarp merge --input /base-estimator/floating_base/state:o /icubSim/chest/inertials/measures:o /base-estimator/contacts/stateAndNormalForce:o --output /odometry_state:o
*/
class YarpOdometryProcessor : public yarp::os::PortReader //yarp::os::BufferedPort<Bottle>
{
private:
    /*Constants*/
    const double sensor_threshold = 100.0;
    const std::string odom_frame_name = "odom";
    const std::string root_link_name = "root_link";

    //Variables
    bool xy_offsets_computed;
    double initial_offset_x, initial_offset_y;
    std::string foot_link;
    nav_msgs::msg::Odometry odom_msg;
    tf2::Quaternion imu_quat;
    geometry_msgs::msg::TransformStamped tf, odom_tf;

public:
    /*Connectivity data */
    std::shared_ptr<OdomPublisher> ros_;
    YarpOdometryProcessor()
    {
        //port.open(server_name);
        //yarp::os::Network::connect(client_name, server_name);     //todo - check for connection or errors
        ros_ = std::make_unique<OdomPublisher>();

        //Ex Odom ros node
        xy_offsets_computed = false;
        this->initial_offset_x = 0;
        this->initial_offset_y = 0;
        /* set up odom msg */
        odom_msg.header.frame_id = this->odom_frame_name;
        odom_msg.child_frame_id = this->root_link_name;
        /* set up odom frame */
        odom_tf.child_frame_id = this->root_link_name;
        odom_tf.header.frame_id = this->odom_frame_name;
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
        this->foot_link = "r_sole"; 
    };

    //using BufferedPort<Bottle>::onRead;
    //void onRead(yarp::os::Bottle data)

    bool read(yarp::os::ConnectionReader& connection) override
    {
        std::cout<< "onRead";
        yarp::os::Bottle data;
        bool ok = data.read(connection);
        if (!ok) {
            std::cout << "Bad Yarp connection \n";
            return false;
        }
        try
        {
            //Compute odom initial offsets on X Y components
            if (! xy_offsets_computed)
            {
                initial_offset_x = data.get(0).asFloat64();
                initial_offset_y = data.get(1).asFloat64();
                xy_offsets_computed = true;
            }
            //Find which foot is in contact with the ground -> for planarity
            if (data.get(19).asFloat64() > sensor_threshold && data.get(20).asFloat64() < sensor_threshold)
            {
                //l_sole
                foot_link = "l_sole"; 
            }
            else if (data.get(19).asFloat64() < sensor_threshold && data.get(20).asFloat64() > sensor_threshold)
            {
                //r_sole
                foot_link = "r_sole"; 
            }
            else if (data.get(19).asFloat64() < sensor_threshold && data.get(20).asFloat64() < sensor_threshold)
            {
                //TODO handle exception of-> when in double support do nothing, keep the previous value
            }

            //Translation Component of odometry X Y Z
            if (! ros_->get_TF("chest", root_link_name, tf))  // updates the TF variable with the most recent tf
            {
                throw std::logic_error("Cannot get TF 1");
            }
            odom_tf.transform.translation.x = data.get(0).asFloat64() - initial_offset_x - tf.transform.translation.x;
            odom_tf.transform.translation.y = data.get(1).asFloat64() - initial_offset_y - tf.transform.translation.y;
            odom_tf.transform.translation.z = data.get(2).asFloat64() + tf.transform.translation.z; 
            //Orientation
            if (! ros_->get_TF(foot_link, root_link_name, tf))  // updates the TF variable with the most recent tf
            {
                throw std::logic_error("Cannot get TF 2");
            }
            tf2::Quaternion tf_ground;    //quat of the root_link to chest frame tf
            tf2::fromMsg(tf.transform.rotation, tf_ground);
            tf2::Matrix3x3 m(tf_ground);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            tf2::Quaternion q_final;  //imu quaternion reading from sensor
            q_final.setRPY(roll, pitch, data.get(12).asList()->get(0).asList()->get(0).asList()->get(2).asFloat64()* 0.0174533);
        

            odom_tf.transform.rotation.x = q_final.x();
            odom_tf.transform.rotation.y = q_final.y();
            odom_tf.transform.rotation.z = q_final.z();
            odom_tf.transform.rotation.w = q_final.w();

            // odom msg
            odom_msg.pose.pose.orientation = odom_tf.transform.rotation;
            odom_msg.pose.pose.position.x = odom_tf.transform.translation.x;
            odom_msg.pose.pose.position.y = odom_tf.transform.translation.y;
            odom_msg.pose.pose.position.z = odom_tf.transform.translation.z;
            odom_msg.twist.twist.linear.x = data.get(6).asFloat64();
            odom_msg.twist.twist.linear.y = data.get(7).asFloat64();
            odom_msg.twist.twist.linear.z = data.get(8).asFloat64();
            odom_msg.twist.twist.angular.x = data.get(9).asFloat64();
            odom_msg.twist.twist.angular.y = data.get(10).asFloat64();
            odom_msg.twist.twist.angular.z = data.get(11).asFloat64();

            // time stamps done when published
            //odom_msg.header.stamp = now(); // yarp::os::Time::now()
            //odom_tf.header.stamp = odom_msg.header.stamp;
            std::cout << "Publishing odom" << "\n";
            ros_->publish_odom(odom_tf, odom_msg);
        }
        catch(const std::exception& e)
        {
            std::cerr << "Error: " << e.what() << "\n";
            return false;
        }
        return true;
    }
};