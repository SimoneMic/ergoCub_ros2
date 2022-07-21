#include "odometry_callback.hpp"

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
    yarp::os::Network::connect(client_name, server_name);

    //create
    auto node = std::make_shared<Estimator_odom>();
    if (rclcpp::ok()) 
    {
        std::cout << "Spinning odometry_standalone node \n";
        rclcpp::spin(node);
    }
    return 0;
}