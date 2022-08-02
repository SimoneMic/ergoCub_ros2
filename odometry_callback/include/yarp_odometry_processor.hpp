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
    const double m_sensor_threshold = 100.0;
    const std::string m_odom_frame_name = "odom";
    const std::string m_root_link_name = "root_link";
    const double m_deg_to_rad = M_PI/180;
    //Variables
    bool m_xy_offsets_computed;
    double m_initial_offset_x, m_initial_offset_y;
    std::string m_foot_link;
    //nav_msgs::msg::Odometry odom_msg;
    //tf2::Quaternion imu_quat;
    //geometry_msgs::msg::TransformStamped tf, odom_tf;

public:
    std::shared_ptr<OdomPublisher> p_ros_;
    YarpOdometryProcessor();
    ~YarpOdometryProcessor();

    bool read(yarp::os::ConnectionReader& connection) override;
};