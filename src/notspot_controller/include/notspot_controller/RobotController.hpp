/*
 *  RobotController.hpp
 *  Author: lnotspotl
 */

#pragma once
#include <Eigen/Core>
#include <sensor_msgs/Imu.h>

#include "notspot_controller/StateCommand.hpp"
#include "notspot_controller/RestController.hpp"
#include "notspot_controller/TrotGaitController.hpp"
#include "notspot_controller/CrawlGaitController.hpp"
#include "notspot_controller/StandController.hpp"

class RobotController
{
    public:

        // CrawlGaitController class constructor - set body and leg dimensions
        RobotController(const float body[], const float legs[]);

        // Main run function, return leg positions in the base_link_world frame
        Eigen::Matrix<float, 3, 4> run();

		// ROS joystick callback
        void joystick_command(const sensor_msgs::Joy::ConstPtr& msg);
        
		// ROS imu callback
        void imu_orientation(const sensor_msgs::Imu::ConstPtr& msg);

        // change current controller if requested
        void change_controller();

        // robot's state
        State state;

    private:
        // variables
        float body[2];
        float legs[4];

        float delta_x;
        float delta_y;
        float x_shift_front;
        float x_shift_back;

        // rest controller
        RestController restController;

        // trot gait controller
        TrotGaitController trotGaitController;

        // crawl gait controller
        CrawlGaitController crawlGaitController;

        // stand controller
        StandController standController;

        Command command;

        // return default_stance
        Eigen::Matrix<float, 3, 4> default_stance();
};
