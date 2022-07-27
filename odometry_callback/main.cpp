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
    //processor.useCallback();
    //processor.open(server_name);
    yarp::os::Network::connect(client_name, server_name);
    std::cout << "setting callback" << std::endl;
    //
    port.setReader(processor);
    std::cout << "passing node" << std::endl;
    auto node = processor.ros_;
    
    if (rclcpp::ok()) 
    {
        std::cout << "Spinning odometry_standalone node" << std::endl;
        rclcpp::spin(node);
    }
    return 0;
}