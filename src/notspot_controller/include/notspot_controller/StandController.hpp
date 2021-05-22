/*
 *  StandController.hpp
 *  Author: lnotspotl
 */

#pragma once
#include <Eigen/Core>
#include <sensor_msgs/Joy.h>

#include "notspot_controller/StateCommand.hpp"

class StandController{
    public:
        // StandController class constructor - set default stance,
        StandController(Eigen::Matrix<float, 3, 4> def_stance);

		// ROS joystick callback - update state and other variables
        void updateStateCommand(const sensor_msgs::Joy::ConstPtr& msg,
                State& state, Command& command);
        
        // Main run function, return leg positions in the base_link_world frame
        Eigen::Matrix<float, 3, 4> run(State& state, Command& command);

    private:
        // robot's default stance
        Eigen::Matrix<float, 3, 4> def_stance;

        // return default_stance
        Eigen::Matrix<float, 3, 4> default_stance();

        // Controller step - return new leg positions
        Eigen::Matrix<float, 3, 4> step(State& state, Command& command);

        // FR leg position in the X direction 
        float FR_X;

        // FR leg position in the Y direction 
        float FR_Y;

        // FL leg position in the X direction 
        float FL_X;
        
        // FL leg position in the Y direction 
        float FL_Y;

        // maximal leg reach
        float max_reach;
};
