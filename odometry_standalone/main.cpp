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
        std::cout << "Spinning \n";
        rclcpp::spin(node);
    }
    return 0;
}