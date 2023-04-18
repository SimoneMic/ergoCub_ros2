#include "rclcpp/rclcpp.hpp"
#include <std_srvs/srv/trigger.hpp>

#include "yarp/os/Bottle.h"
#include "yarp/os/Port.h"
#include "yarp/os/Network.h"

#include <memory>


std::atomic<bool> state = true;

void is_on_double_support(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response>      response)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[is_on_double_support_srv] Received a request - state: %i", (int)state);
    response->success = state;
    state = false;  //reset the variable each time is resetted
}

//Class used for YARP port callbacks.
class YarpTriggerProcessor : public yarp::os::PortReader
{
private:
    std::mutex m_mutex;


public:
    YarpTriggerProcessor()
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[is_on_double_support_srv] Created YarpTriggerProcessor");
    };

    //main loop executed for each port reading of the merged feet status
    bool read(yarp::os::ConnectionReader& t_connection) override
    {
        try
        {
            //std::lock_guard<std::mutex> guard(m_mutex);
            yarp::os::Bottle b;
            bool ok = b.read(t_connection);
            if (!ok) {
                std::cout << "Bad Yarp connection " << std::endl;
                return false;
            }

        
            state = b.get(0).asBool();
            if (state)
            {
                std::cout << "[is_on_double_support_srv] Red a trigger on YARP port" << std::endl;
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << std::endl;
        }
        return true;
    }
};  //End of class YarpTriggerProcessor 

int main(int argc, char **argv)
{
    const std::string portName = "/planner_trigger_server/reader:i";
    const std::string sourceName = "/navigation_helper/replanning_trigger:o";
    rclcpp::init(argc, argv);
    yarp::os::Network yarp;
    yarp::os::Port port;

    if (rclcpp::ok())
    {
        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("is_on_double_support_srv");
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service = node->create_service<std_srvs::srv::Trigger>
                                                                     ("is_on_double_support_srv", &is_on_double_support);
        //Connect yarp ports
        port.open(portName);
        yarp::os::Network::connect(sourceName, portName);
        YarpTriggerProcessor processor;

        if(yarp::os::Network::isConnected(sourceName, portName)){
            port.setReader(processor);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[is_on_double_support_srv] YARP Ports connected successfully");
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[is_on_double_support_srv] Error connecting with YARP ports: %s with %s - Shutting Down", sourceName, portName);
            return 1;
        }

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[is_on_double_support_srv] Spinning service node");
        rclcpp::spin(node);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[is_on_double_support_srv] Shutting down");
        rclcpp::shutdown();
    }
    
    
}
