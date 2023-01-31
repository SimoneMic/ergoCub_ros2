/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra, Simone Micheletti
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "/home/user1/robotology-superbuild/build/install/include/UnicyclePlanner.h"
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/Utils.h>
#include </usr/include/eigen3/Eigen/Core>
#include "iDynTree/Core/EigenHelpers.h"
#include "iDynTree/Core/MatrixDynSize.h"
#include <cmath>
#include <memory>
#include <iostream>
#include <ctime>
 
// Custom ROS2
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <mutex>

using namespace std::chrono_literals;
using std::placeholders::_1;

/*****************************************************************************************************************************/

class RosNode : public rclcpp::Node
{
private:
    std::mutex m_mutex;
    /* consts */
    const std::string m_sub_topic_name = "/plan";
    const std::string m_pathPub_topic_name = "/unicycle_path_follower/dcm_path";
    const std::string m_ritgh_footprints_topic_name = "/unicycle_path_follower/right_footprints";
    const std::string m_left_footprints_topic_name = "/unicycle_path_follower/left_footprints";
    const std::string m_robot_frame = "virtual_unicycle_base";
    //const std::string m_world_frame = "map";
    //pubs and subs
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_dcm_pub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr m_path_sub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_right_footprint_markers_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_left_footprint_markers_pub;
    // TFs
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer_in;

    //msg
    nav_msgs::msg::Path::ConstPtr m_path_msg;
    
    //debug
    bool debug_once = false;

    void sub_callback(const nav_msgs::msg::Path::ConstPtr &msg_in)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        try
        {
            if (debug_once)
            {
                RCLCPP_INFO(this->get_logger(), "Quitting callback");
                return;
            }
            //debug_once = true;
            m_path_msg = msg_in;
            // Each time a path is published I need to transform it to the robot frame
            std::cout << "msg_in->header.frame_id: " << msg_in->header.frame_id << std::endl;
            if (msg_in->header.frame_id == m_robot_frame)
            {
                plannerTest(*msg_in);
            }
            else
            {
                geometry_msgs::msg::TransformStamped tf = m_tf_buffer_in->lookupTransform(m_robot_frame, msg_in->header.frame_id, rclcpp::Time(0));
                nav_msgs::msg::Path transformed_path = transformPlan(tf, m_path_msg, false);

                plannerTest(transformed_path);
            }
            
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to transform global path to robot frame");
        }
    }

    nav_msgs::msg::Path transformPlan(geometry_msgs::msg::TransformStamped & tf_, nav_msgs::msg::Path::ConstPtr &untransformed_path, bool prune_plan = false){
        if (untransformed_path->poses.empty()) {
            std::cerr << "Received plan with zero length" << std::endl;
            throw std::runtime_error("Received plan with zero length");
        }
        
        // let's get the pose of the robot in the frame of the plan

        // Transform the near part of the global plan into the robot's frame of reference.
        nav_msgs::msg::Path transformed_plan_ = *untransformed_path;
        transformed_plan_.header.frame_id = m_robot_frame;
        transformed_plan_.header.stamp = untransformed_path->header.stamp;  //could be removed

        //Transform the whole path (we could transform the path up to a certain point to save resources)
        //std::cout << "Transform the whole path for loop" << std::endl;
        for (int i = 0; i < untransformed_path->poses.size(); ++i)
        {
            tf2::doTransform(m_path_msg->poses.at(i), transformed_plan_.poses.at(i), tf_);
            tf2::Quaternion tmp_quat;
            tf2::fromMsg(transformed_plan_.poses.at(i).pose.orientation, tmp_quat);
            double roll, pitch, yaw;
            tf2::Matrix3x3 m(tmp_quat);
            m.getRPY(roll, pitch, yaw);
            std::cout << "Transformed X: " << transformed_plan_.poses.at(i).pose.position.x << "  Y: " << transformed_plan_.poses.at(i).pose.position.y <<
            " Theta: " << yaw <<std::endl;
        }
        
        // Remove the portion of the global plan that is behind the robot
        if (prune_plan) {
            // Helper predicate lambda function to see what is the positive x-element in a vector of poses
            // Warning: The robot needs to have a portion of the path that goes forward (positive X)
            auto greaterThanZero = [](const geometry_msgs::msg::PoseStamped &i){
                return i.pose.position.x > 0.0;
            };

            transformed_plan_.poses.erase(begin(transformed_plan_.poses), 
                                        std::find_if(transformed_plan_.poses.begin(),
                                                transformed_plan_.poses.end(),
                                                greaterThanZero));
        }
        
        if (transformed_plan_.poses.empty()) {
            std::cerr << "Resulting plan has 0 poses in it." << std::endl;
            throw std::runtime_error("Resulting plan has 0 poses in it");
        }
        return transformed_plan_;
    }

    bool populateDesiredPath(UnicyclePlanner& planner, double initTime, double endTime, double dT, const nav_msgs::msg::Path &path, const double granularity=0.001){

        double t = initTime;    //0.0
        iDynTree::Vector2 yDes, yDotDes, polarCoordinates;    //yDes is the x, y pose 
        size_t index = 1;   //skip the first pose (should be too close to the robot origin and could be ignored)
        std::vector<iDynTree::Vector2> poses_history;
        // In theory, the first pose should be the reference frame in (0,0)
        double slope_angle = std::atan2(path.poses.at(index).pose.position.y, path.poses.at(index).pose.position.x);
        double speed = granularity/dT;

        while (t <= endTime){ 
            //Calculate the points separated by a fixed granularity
            if (poses_history.empty())
            {
                yDes(0) = granularity * std::cos(slope_angle);
                yDes(1) = granularity * std::sin(slope_angle);
                poses_history.push_back(yDes);
            }
            else
            {
                yDes(0) = granularity * std::cos(slope_angle) + poses_history.back()(0);
                yDes(1) = granularity * std::sin(slope_angle) + poses_history.back()(1);
                poses_history.push_back(yDes);
            }
            // Velocities
            yDotDes(0) = speed * std::cos(slope_angle);
            yDotDes(1) = speed * std::sin(slope_angle);
            
            // check if the next goal pose of the path is reached
            if (nextPoseReached(path, yDes, index, slope_angle))
            {
                ++index;
                //check if last posed is reached
                if (index >= path.poses.size() - 1)
                {
                    if(!planner.addPersonFollowingDesiredTrajectoryPoint(t, yDes, yDotDes))
                        return false;
                    break;
                }
                //update the slope angle with the direction of the next pose in the path
                //TODO check if use the latest pose from path or from poses_history
                slope_angle = std::atan2(path.poses.at(index + 1).pose.position.y - path.poses.at(index).pose.position.y, 
                                         path.poses.at(index + 1).pose.position.x - path.poses.at(index).pose.position.x);
            }
            
            if(!planner.addPersonFollowingDesiredTrajectoryPoint(t, yDes, yDotDes))
                return false;

            t += dT;
        }
        return true;
    }

    bool nextPoseReached (const nav_msgs::msg::Path &path, const iDynTree::Vector2 &pose, const size_t &current_index, const double &slope_angle){
        double distance_x = std::abs(path.poses.at(current_index).pose.position.x * std::cos(slope_angle)) -
                            std::abs(pose(0) * std::cos(slope_angle));
        double distance_y = std::abs(path.poses.at(current_index).pose.position.y * std::sin(slope_angle)) -
                            std::abs(pose(1) * std::sin(slope_angle));
        // condition if I am matching the pose or overshooting it
        if (distance_x >= 0 && distance_y >=0)
            return true;
        else
            return false;
    }

    bool setWaypoints(UnicyclePlanner& planner, double initTime, double endTime, const nav_msgs::msg::Path &path, const double speed){
        //setting init pose to 0 0 0
        planner.clearPersonFollowingDesiredTrajectory();
        iDynTree::Vector2 initPose;
        initPose(0) = 0.0;
        initPose(1) = 0.0;
        planner.addPersonFollowingDesiredTrajectoryPoint(initTime, initPose);
        //Skipping first pose from path (replaced by previous zeroes)
        double elapsed_time = 0.0;
        iDynTree::Vector2 desiredSpeed;
        desiredSpeed(0) = speed;
        desiredSpeed(1) = .0;
        for (size_t i = 1 ; i < path.poses.size(); ++i)
        {
            std::cout << i << std::endl;
            double distance = sqrt(pow(path.poses.at(i).pose.position.x - path.poses.at(i-1).pose.position.x, 2) + 
                                   pow(path.poses.at(i).pose.position.y - path.poses.at(i-1).pose.position.y, 2));
            double eta = distance / speed;
            elapsed_time += eta;  //add a pause for caution?
            //check data
            if (std::isnan(elapsed_time))
            {
                RCLCPP_ERROR(this->get_logger(), "Error - NaN time: %f with: i = %i pose(i) = (%f, %f) pose(i-1) = (%f, %f)", 
                            elapsed_time, i, path.poses.at(i).pose.position.x, path.poses.at(i).pose.position.y, path.poses.at(i-1).pose.position.x, path.poses.at(i-1).pose.position.y);
                return false;   
            }
            
            if (elapsed_time > endTime)   // exit condition
            {
                RCLCPP_INFO(this->get_logger(), "Exiting at No: %f", i);
                break;
            }
            RCLCPP_INFO(this->get_logger(), "Setting waypont Time: %f ", elapsed_time);
            iDynTree::Vector2 yDes;
            yDes(0) = path.poses.at(i).pose.position.x;   //+ 0.05  should I add the person following distance?
            yDes(1) = path.poses.at(i).pose.position.y; 

            planner.addPersonFollowingDesiredTrajectoryPoint(elapsed_time, yDes, desiredSpeed);  
        }
        return true;
    }


struct Configuration {
    double initTime = 0.0, endTime = 50.0, dT = 0.01, K = 10, dX = 0.2, dY = 0.0;
    double maxL = 0.2, minL = 0.05, minW = 0.08, maxAngle = iDynTree::deg2rad(45), minAngle = iDynTree::deg2rad(5);
    double nominalW = 0.14, maxT = 10, minT = 3, nominalT = 4, timeWeight = 2.5, positionWeight = 1;
    bool swingLeft = true;
    double slowWhenTurnGain = 0.5;
};

bool printSteps(std::deque<Step> leftSteps, std::deque<Step> rightSteps){
    std::cerr << "Left foot "<< leftSteps.size() << " steps:"<< std::endl;
    for (auto step : leftSteps){
        std::cerr << "Position "<< step.position.toString() << std::endl;
        std::cerr << "Angle "<< iDynTree::rad2deg(step.angle) << std::endl;
        std::cerr << "Time  "<< step.impactTime << std::endl;
    }


    std::cerr << std::endl << "Right foot "<< rightSteps.size() << " steps:" << std::endl;
    for (auto step : rightSteps){
        std::cerr << "Position "<< step.position.toString() << std::endl;
        std::cerr << "Angle "<< iDynTree::rad2deg(step.angle) << std::endl;
        std::cerr << "Time  "<< step.impactTime << std::endl;
    }

    return true;
}

bool checkConstraints(std::deque<Step> leftSteps, std::deque<Step> rightSteps, Configuration conf){
    //Checking constraints

    conf.swingLeft = (leftSteps.front().impactTime >= rightSteps.front().impactTime);

    if (leftSteps.front().impactTime == rightSteps.front().impactTime)
        leftSteps.pop_front(); //this is a fake step!

    bool result = true;
    double distance = 0.0, deltaAngle = 0.0, deltaTime = 0.0, c_theta, s_theta;
    iDynTree::MatrixDynSize rTranspose(2,2);
    iDynTree::Vector2 rPl;

    while (!leftSteps.empty() && !rightSteps.empty()){
        distance = (iDynTree::toEigen(leftSteps.front().position) - iDynTree::toEigen(rightSteps.front().position)).norm();

        if (distance > conf.maxL){
            std::cerr <<"[ERROR] Distance constraint not satisfied" << std::endl;
            result = false;
        }

        deltaAngle = std::abs(leftSteps.front().angle - rightSteps.front().angle);

        if (deltaAngle > conf.maxAngle){
            std::cerr <<"[ERROR] Angle constraint not satisfied" << std::endl;
            result = false;
        }

        deltaTime = std::abs(leftSteps.front().impactTime - rightSteps.front().impactTime);

        if (deltaTime < conf.minT){
            std::cerr <<"[ERROR] Min time constraint not satisfied" << std::endl;
            result = false;
        }

        c_theta = std::cos(rightSteps.front().angle);
        s_theta = std::sin(rightSteps.front().angle);

        rTranspose(0,0) = c_theta;
        rTranspose(1,0) = -s_theta;
        rTranspose(0,1) = s_theta;
        rTranspose(1,1) = c_theta;

        iDynTree::toEigen(rPl) =
                iDynTree::toEigen(rTranspose)*(iDynTree::toEigen(leftSteps.front().position) - iDynTree::toEigen(rightSteps.front().position));

        if (rPl(1) < conf.minW){
            std::cerr <<"[ERROR] Width constraint not satisfied: " << rPl(1) << " at TIME: " << leftSteps.front().impactTime << std::endl;
            result = false;
        }

        if(conf.swingLeft)
            rightSteps.pop_front();
        else leftSteps.pop_front();

        conf.swingLeft = !conf.swingLeft;

    }

    if(!result)
        return false;

    return true;
}
     
    bool plannerTest(const nav_msgs::msg::Path &path){
        Configuration conf;
        conf.initTime = 0.0;
        conf.endTime = 50.0;    //50.0
        conf.dT = 0.01;
        conf.K = 10;
        conf.dX = 0.05;  
        conf.dY = 0.0;
        conf.maxL = 0.21;    //0.2
        conf.minL = 0.05;
        conf.minW = 0.08;      //0.08 -> 0.12
        conf.maxAngle = iDynTree::deg2rad(22);  //45
        conf.minAngle = iDynTree::deg2rad(0);   //5 -> 8
        conf.nominalW = 0.14;
        conf.maxT = 1.3;     //10     
        conf.minT = 0.7;      //1.1
        conf.nominalT = 1.2;  //4
        conf.timeWeight = 2.5;
        conf.positionWeight = 1;
        conf.swingLeft = true;
        conf.slowWhenTurnGain = 5.0;    //0.5

        UnicyclePlanner planner;

        //Initialization (some of these calls may be avoided)
        iDynTree::assertTrue(planner.setDesiredPersonDistance(conf.dX, conf.dY));
        iDynTree::assertTrue(planner.setPersonFollowingControllerGain(conf.K));
        iDynTree::assertTrue(planner.setMaximumIntegratorStepSize(conf.dT));
        iDynTree::assertTrue(planner.setMaxStepLength(conf.maxL));
        iDynTree::assertTrue(planner.setWidthSetting(conf.minW, conf.nominalW));
        iDynTree::assertTrue(planner.setMaxAngleVariation(conf.maxAngle));
        iDynTree::assertTrue(planner.setCostWeights(conf.positionWeight, conf.timeWeight));
        iDynTree::assertTrue(planner.setStepTimings(conf.minT, conf.maxT, conf.nominalT));
        iDynTree::assertTrue(planner.setPlannerPeriod(conf.dT));
        iDynTree::assertTrue(planner.setMinimumAngleForNewSteps(conf.minAngle));
        iDynTree::assertTrue(planner.setMinimumStepLength(conf.minL));
        iDynTree::assertTrue(planner.setSlowWhenTurnGain(conf.slowWhenTurnGain));
        iDynTree::assertTrue(planner.setSaturationsConservativeFactors(0.7, 0.7));
        //ADD THIS IF USING INTERPOLATION FOR THE COMPUTATION OF NEW STEPS
        iDynTree::assertTrue(planner.setUnicycleController(UnicycleController::DIRECT));
        planner.setDesiredDirectControl(10.0, 0.0, 0.0);

        planner.addTerminalStep(true);
        planner.startWithLeft(conf.swingLeft);
        
 
        //Generate desired trajectory
        clock_t start = clock();

        std::vector<UnicycleState> converted_path;
        UnicycleState tmp_pose;
        for (size_t i = 0; i < path.poses.size(); ++i)
        {
            tf2::Quaternion q;
            tf2::fromMsg(path.poses.at(i).pose.orientation, q);
            double roll, pitch, yaw;
            tf2::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);
            tmp_pose.angle = yaw;
            tmp_pose.position(0) = path.poses[i].pose.position.x;
            tmp_pose.position(1) = path.poses[i].pose.position.y;
            converted_path.push_back(tmp_pose);
        } 
        planner.setInputPath(converted_path);
        //double approx_speed = std::sqrt(std::pow(conf.maxL, 2) - std::pow(conf.nominalW, 2)) / conf.minT * 0.9 * 0.8;   //from ComputeNewSteps in UnicyclePlanner.cpp
        //std::cerr <<"APPROX SPEED: " << approx_speed <<std::endl;
        //iDynTree::assertTrue(populateDesiredTrajectory(planner, conf.initTime, conf.endTime, conf.dT));
        //iDynTree::assertTrue(setWaypoints(planner, conf.initTime, conf.endTime, path, approx_speed));
        //std::cerr <<"Populating the trajectory took " << (static_cast<double>(clock() - start) / CLOCKS_PER_SEC) << " seconds."<<std::endl;

        std::shared_ptr<FootPrint> left, right;
        left = std::make_shared<FootPrint>();
        right = std::make_shared<FootPrint>();

        start = clock();
        //iDynTree::assertTrue(planner.computeNewSteps(left, right, conf.initTime, conf.endTime));
        iDynTree::assertTrue(planner.interpolateNewStepsFromPath(left, right, conf.initTime, conf.endTime));
        std::cerr <<"Test Finished in " << (static_cast<double>(clock() - start) / CLOCKS_PER_SEC) << " seconds."<<std::endl;

        StepList leftSteps = left->getSteps();
        StepList rightSteps = right->getSteps();

        std::cerr << "First test." << std::endl;
        // publish markers on ros2
        RCLCPP_INFO(this->get_logger(), "Publishing markers");
        publishMarkers(leftSteps, rightSteps);
        iDynTree::assertTrue(printSteps(leftSteps, rightSteps));
        //iDynTree::assertTrue(checkConstraints(leftSteps, rightSteps, conf));

        
        /*
        std::cerr << std::endl << "------------------------------------------------------------------" << std::endl;
        std::cerr << "Second test." << std::endl;

        left->clearSteps();
        iDynTree::assertTrue(right->dropPastSteps());
        iDynTree::assertTrue(right->numberOfSteps() == 1);
        Step lastStep;
        iDynTree::assertTrue(right->getLastStep(lastStep));
        planner.clearPersonFollowingDesiredTrajectory();
        iDynTree::Vector2 dummyVector, newDesired;
        dummyVector.zero();
        newDesired(0) = lastStep.position(0) + 0.5;
        newDesired(1) = lastStep.position(1) + 0.5;
        iDynTree::assertTrue(planner.addPersonFollowingDesiredTrajectoryPoint(lastStep.impactTime+10, newDesired, dummyVector));

        iDynTree::assertTrue(planner.computeNewSteps(left, right, lastStep.impactTime, lastStep.impactTime+10));

        leftSteps = left->getSteps();
        rightSteps = right->getSteps();

        iDynTree::assertTrue(printSteps(leftSteps, rightSteps));

        iDynTree::assertTrue(checkConstraints(leftSteps, rightSteps, conf));
        */
        return true;
    }

    bool publishMarkers(std::deque<Step> leftSteps, std::deque<Step> rightSteps){
        if (leftSteps.size()==0 || rightSteps.size()==0)
        {
            RCLCPP_INFO(this->get_logger(), "One of the Step array is empty");
            return false;
        }
        
        visualization_msgs::msg::MarkerArray right_marker_array;
        visualization_msgs::msg::MarkerArray left_marker_array;
        visualization_msgs::msg::Marker tmp_marker_msg;
        builtin_interfaces::msg::Time timestamp = now();
        //LEFT
        RCLCPP_INFO(this->get_logger(), "Left Loop");
        for (size_t i = 0; i < leftSteps.size(); ++i)
        {
            visualization_msgs::msg::Marker tmp_marker_msg;
            tmp_marker_msg.header.frame_id = "virtual_unicycle_base";
            tmp_marker_msg.id = i;
            tmp_marker_msg.header.stamp = timestamp;
            tmp_marker_msg.scale.x = 0.05;
            tmp_marker_msg.scale.y = 0.05;
            tmp_marker_msg.scale.z = 0.05;
            // Color for left foot
            tmp_marker_msg.color.r = 0.0;
            tmp_marker_msg.color.g = 1.0;
            tmp_marker_msg.color.b = 0.0;
            tmp_marker_msg.color.a = 1.0;
            tmp_marker_msg.type = visualization_msgs::msg::Marker::ARROW;
            tmp_marker_msg.pose.position.x = leftSteps.at(i).position(0);
            tmp_marker_msg.pose.position.y = leftSteps.at(i).position(1);
            tmp_marker_msg.pose.position.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, leftSteps.at(i).angle);
            tmp_marker_msg.pose.orientation = tf2::toMsg(q);
            tmp_marker_msg.frame_locked = true;
            tmp_marker_msg.action = visualization_msgs::msg::Marker::ADD;
            //Populate the marker with atleast one mesh point
            geometry_msgs::msg::Point cube_center;
            cube_center.x = 0.0;
            cube_center.y = 0.0;
            cube_center.z = 0.0;
            tmp_marker_msg.points.push_back(cube_center);
            cube_center.x = 0.1;
            cube_center.y = 0.0;
            cube_center.z = 0.0;
            tmp_marker_msg.points.push_back(cube_center);
            //save marker in the array
            left_marker_array.markers.push_back(tmp_marker_msg);
        }
        RCLCPP_INFO(this->get_logger(), "Publishing Left");
        m_left_footprint_markers_pub->publish(left_marker_array);
        //for (auto it = leftSteps.begin(); it != leftSteps.end(); ++it)
        //{
        //    tmp_marker_msg.id = std::distance(leftSteps.begin(), it); 
        //    tmp_marker_msg.pose.position.x = it->position(0);
        //    tmp_marker_msg.pose.position.y = it->position(1);
        //    tf2::Quaternion q;
        //    q.setRPY(0, 0, it->angle);
        //    tmp_marker_msg.pose.orientation = tf2::toMsg(q);
//
        //    left_marker_array.markers.push_back(tmp_marker_msg);
        //}

        //RIGHT
        RCLCPP_INFO(this->get_logger(), "Right Loop");
        /*
        for (auto it = rightSteps.begin(); it != rightSteps.end(); ++it)
        {
            tmp_marker_msg.id = std::distance(rightSteps.begin(), it); 
            tmp_marker_msg.pose.position.x = it->position(0);
            tmp_marker_msg.pose.position.y = it->position(1);
            tf2::Quaternion q;
            q.setRPY(0, 0, it->angle);
            tmp_marker_msg.pose.orientation = tf2::toMsg(q);

            right_marker_array.markers.push_back(tmp_marker_msg);
        } */
        tmp_marker_msg.points.clear();
        for (size_t i = 0; i < rightSteps.size(); ++i)
        {
            visualization_msgs::msg::Marker tmp_marker_msg;
            tmp_marker_msg.header.frame_id = "virtual_unicycle_base";
            tmp_marker_msg.id = i;
            tmp_marker_msg.header.stamp = timestamp;
            tmp_marker_msg.scale.x = 0.05;
            tmp_marker_msg.scale.y = 0.05;
            tmp_marker_msg.scale.z = 0.05;
            // Color for left foot
            tmp_marker_msg.color.r = 1.0;
            tmp_marker_msg.color.g = 0.0;
            tmp_marker_msg.color.b = 0.0;
            tmp_marker_msg.color.a = 1.0;
            tmp_marker_msg.type = visualization_msgs::msg::Marker::ARROW;
            tmp_marker_msg.pose.position.x = rightSteps.at(i).position(0);
            tmp_marker_msg.pose.position.y = rightSteps.at(i).position(1);
            tmp_marker_msg.pose.position.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, rightSteps.at(i).angle);
            tmp_marker_msg.pose.orientation = tf2::toMsg(q);
            tmp_marker_msg.frame_locked = true;
            tmp_marker_msg.action = visualization_msgs::msg::Marker::ADD;
            //Populate the marker with atleast 2 mesh point for ARROW
            geometry_msgs::msg::Point cube_center;
            cube_center.x = 0.0;
            cube_center.y = 0.0;
            cube_center.z = 0.0;
            tmp_marker_msg.points.push_back(cube_center);
            cube_center.x = 0.1;
            cube_center.y = 0.0;
            cube_center.z = 0.0;
            tmp_marker_msg.points.push_back(cube_center);
            //save marker in the array
            right_marker_array.markers.push_back(tmp_marker_msg);
        }

        RCLCPP_INFO(this->get_logger(), "Publishing Right");
        //publish
        m_right_footprint_markers_pub->publish(right_marker_array);
    }

public:
    RosNode() : rclcpp::Node("unicycle_path_follower_node")
    {
        // pubs and subs
        m_dcm_pub = this->create_publisher<nav_msgs::msg::Path>(m_pathPub_topic_name, 10);
        m_right_footprint_markers_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(m_ritgh_footprints_topic_name, 10);
        m_left_footprint_markers_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(m_left_footprints_topic_name, 10);
        m_path_sub = this->create_subscription<nav_msgs::msg::Path>(m_sub_topic_name,
                                                                    10,
                                                                    std::bind(&RosNode::sub_callback, this, _1)
                                                                    );
        //TFs
        m_tf_buffer_in = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_in);

    }
    
};

/*****************************************************************************************************************************/





int main(int argc, char **argv) {
    //iDynTree::assertTrue(plannerTest());
    //std::cerr << "----------------Direct Control Test -------------------" << std::endl;
    //iDynTree::assertTrue(directControlTest());
    // ROS
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RosNode>();
    if (rclcpp::ok()) 
    {
        std::cout << "Spinning ROS2 node" << std::endl;
        rclcpp::spin(node);
        std::cout << "Shutting down" << std::endl;
        rclcpp::shutdown();
    }
    return EXIT_SUCCESS;
}
