#include "yarp_odometry_processor.hpp"

int main(int argc, char* argv[])
{
    // ROS
    rclcpp::init(argc, argv);
    // YARP network
    yarp::os::Network yarp;
    //create yarp ports
    const std::string client_name = "/odometry_state:o";
    const std::string server_name = "/odometry_processor/odom_data:i";
    yarp::os::Port port;
    port.open(server_name);
    YarpOdometryProcessor processor;
    yarp::os::Network::connect(client_name, server_name);
    std::cout << "setting callback" << std::endl;
    port.setReader(processor);
    std::cout << "passing node" << std::endl;
    auto node = processor.p_ros_;
    
    if (rclcpp::ok()) 
    {
        std::cout << "Spinning odometry_standalone node" << std::endl;
        rclcpp::spin(node);
        std::cout << "Shutting down" << std::endl;
        rclcpp::shutdown();
    }
    return 0;
}