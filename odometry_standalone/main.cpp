#include "odometry_standalone.hpp"

int main(int argc, char* argv[])
{
    // YARP network
    yarp::os::Network yarp;
    // ROS
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Estimator_odom>();
    if (rclcpp::ok()) 
    {
        std::cout << "Spinning odometry_standalone node" << std::endl;
        rclcpp::spin(node);
        std::cout << "Shutting down" << std::endl;
        rclcpp::shutdown();
    }
    return 0;
}