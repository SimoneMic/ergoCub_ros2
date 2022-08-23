#include <yarp/os/Bottle.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/RpcClient.h>

#include <cstdlib>
#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <list>

bool ok = true; //Global flag for main while loop

void sig_handler(int s)
{
    std::cout << "Caught: " << s << std::endl;
    ok = false;
    //exit(1); 
}

int main(int argc, char const *argv[])
{
    //YARP ports setup
    yarp::os::Network yarp;
    const std::string client_name = "/obstacle_script/client";
    const std::string srv_name = "/world_input_port";
    yarp::os::RpcClient rpcPort;
    rpcPort.open(client_name);
    yarp::os::Network::connect(client_name,srv_name);

    //Sig Handler -> Catch Ctrl-C
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = sig_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    //Geti initial pose of obstacle
    bool initialized = false;
    const std::string obstacle_name = "unit_cylinder";
    struct Pose
    {
        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;
        std::list<double> poses = {x, y, z, roll, pitch, yaw};
    };
    Pose object_pose;
    double increment = 0.001;   //distance increment (in m) added to the y-pose of the obstacle at each loop
    const double step = 0.001;
    bool direction = true;  //if true it's positive increment, negative otherwise
    //Main loop until Ctrl-C is caught
    while (ok)
    {
        yarp::os::Bottle cmd, response;     //Need to spawn stickBot before talking with the worldinterface port -> it's that model that has the worldinterface
        if (!initialized)
        {
            cmd.addString("getPose");
            cmd.addString(obstacle_name);
            rpcPort.write(cmd, response);
            std::cout << "Pose: " << response.get(0).asFloat64() << " " << 
                        response.get(1).asFloat64() << " " <<
                        response.get(2).asFloat64() << std::endl;
            object_pose.x = response.get(0).asFloat64();
            object_pose.y = response.get(1).asFloat64();
            //object_pose.z = response.get(2).asFloat64();
            object_pose.z = 0.5;
            object_pose.roll = response.get(3).asFloat64();
            object_pose.pitch = response.get(4).asFloat64();
            object_pose.yaw = response.get(5).asFloat64();
            initialized = true;
        }
        else
        {
            cmd.addString("setPose");
            cmd.addString(obstacle_name);
            cmd.addFloat64(object_pose.x);
            cmd.addFloat64(object_pose.y + increment);
            cmd.addFloat64(object_pose.z);
            cmd.addFloat64(object_pose.roll);
            cmd.addFloat64(object_pose.pitch);
            cmd.addFloat64(object_pose.yaw);
            rpcPort.write(cmd, response);
            std::cout << "Responded: " << response.get(0).asString() << std::endl;

            if (direction)
            {
                increment += step;
            }
            else
            {
                increment -= step;
            }
            
            if (increment >= 3.0)
            {
                direction = false;
            }
            else if (increment <= -3.0)
            {
                direction = true;
            }
        }
        sleep(0.1);
    }
    std::cout << "Closing..." << std::endl;

    return 0;
}
