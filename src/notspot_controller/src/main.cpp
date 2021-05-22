/*
 *  main.cpp
 *  Author: lnotspotl
 */

#include <ros/ros.h>
#include <Eigen/Dense>
#include "notspot_controller/InverseKinematics.hpp"
#include "notspot_controller/Transformations.hpp"
#include "notspot_controller/RobotController.hpp"
#include <vector>
#include <string>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>

#define RATE 60
int main(int argc, char* argv[])
{
    // robot body dimensions - body length, body width
    const float body_dimensions[] = {0.1908, 0.080};

    // robot leg dimensions - l1, l2, l3, l4
    const float leg_dimensions[] = {0.0, 0.04, 0.100, 0.094333};

    // gazebo command topics
    std::string command_topics[] = 
    {
          "/notspot_controller/FR1_joint/command",
          "/notspot_controller/FR2_joint/command",
          "/notspot_controller/FR3_joint/command",
          "/notspot_controller/FL1_joint/command",
          "/notspot_controller/FL2_joint/command",
          "/notspot_controller/FL3_joint/command",
          "/notspot_controller/RR1_joint/command",
          "/notspot_controller/RR2_joint/command",
          "/notspot_controller/RR3_joint/command",
          "/notspot_controller/RL1_joint/command",
          "/notspot_controller/RL2_joint/command",
          "/notspot_controller/RL3_joint/command",
    };

    
    // ROS node initialization
    ros::init(argc, argv, "Robot_Controller");
    ros::NodeHandle node_handle;

    // RobotController
    RobotController notspot(body_dimensions, leg_dimensions);
    ros::Subscriber dsd = node_handle.subscribe("notspot_joy/joy_ramped", 1,
            &RobotController::joystick_command, &notspot);
    ros::Subscriber dsa = node_handle.subscribe("notspot_imu/base_link_orientation", 1,
            &RobotController::imu_orientation, &notspot);

    // Inverse Kinematics 
    InverseKinematics notspot_IK(body_dimensions, leg_dimensions);

    // Gazebo command publishers
    std::vector<ros::Publisher> publishers;
    for(int i = 0; i < 12; i++)
    {
        ros::Publisher new_publisher = node_handle.advertise<std_msgs::Float64>
            (command_topics[i],1);
        publishers.push_back(new_publisher);
    }

    // main while loop rate
    ros::Rate loop_rate(RATE);
    while(ros::ok())
    {
        // new leg positions
        Eigen::Matrix<float, 3, 4> leg_positions = notspot.run();
        notspot.change_controller();

        // body local position
        float dx = notspot.state.body_local_position[0];
        float dy = notspot.state.body_local_position[1];
        float dz = notspot.state.body_local_position[2];

        // body local orientation
        float roll = notspot.state.body_local_orientation[0];
        float pitch = notspot.state.body_local_orientation[1];
        float yaw = notspot.state.body_local_orientation[2];

        // inverse kinematics -> joint angles
        std::vector<double> angles = notspot_IK.inverse_kinematics(leg_positions,
                dx, dy, dz, roll, pitch, yaw);

        // publish joint angle commands
        for(int i = 0; i < 12; i++)
        {
            if(!isnan(angles[i]))
            {
                std_msgs::Float64 command_message;
                command_message.data = angles[i];
                publishers[i].publish(command_message);
            }
        }
        
        // spin
        ros::spinOnce();
        
        // sleep
        loop_rate.sleep();
    }

    return 0;
}
