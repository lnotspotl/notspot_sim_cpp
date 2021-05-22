/*
 *  RestController.cpp
 *  Author: lnotspotl
 */

#include "notspot_controller/RestController.hpp"
#include "notspot_controller/StateCommand.hpp"
#include "notspot_controller/Transformations.hpp"

#include <cmath>
#include <iostream>

#define MAX_TILT 0.6

///////////////////////////////////////////////////////////////////////////////
RestController::RestController(Eigen::Matrix<float, 3, 4> default_stance)
: pid_controller(0.75, 2.29, 0.00)
// pid_controller(0.74, 2.49, 0.00)
//pid_controller(0.72, 0.99, 0.00)
{
    def_stance = default_stance;
    use_button = true;
    use_imu = false;
    max_tilt = MAX_TILT;
}

///////////////////////////////////////////////////////////////////////////////
void RestController::updateStateCommand(const sensor_msgs::Joy::ConstPtr& msg,
        State& state, Command& command)
{
    // robot body local position
    state.body_local_position[0] = msg->axes[7] * 0.04; 
    state.body_local_position[1] = msg->axes[6] * 0.03; 
    state.body_local_position[2] = msg->axes[1] * 0.03; 

    // robot body local orientation
    state.body_local_orientation[0] = msg->axes[0] * 0.4;
    state.body_local_orientation[1] = msg->axes[4] * 0.5;
    state.body_local_orientation[2] = msg->axes[3] * 0.4;

    if(use_button){
        if(msg->buttons[7]){
            use_imu = !use_imu;
            use_button = false;
            if(use_imu)
                ROS_INFO("Rest Controller - Use roll/pitch compensation"
                        " : " "true");
            else
                ROS_INFO("Rest Controller - Use roll/pitch compensation"
                        " : " "false");
        }
    }

    if(!use_button)
    {
        if(!(msg->buttons[7]))
            use_button = true;
    }
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float, 3, 4> RestController::default_stance()
{
    return def_stance;
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float, 3, 4> RestController::step(State& state, Command& command)
{
    Eigen::Matrix<float, 3, 4> temp = default_stance();
    for(int i = 0; i < 4; i++)
    {
        temp(2,i) = command.robot_height;
    }

    if(use_imu)
    {
        Eigen::Vector2f compensation;
        compensation = pid_controller.run(state.imu_roll, state.imu_pitch);
        float roll_comp = -compensation[0];
        float pitch_comp = -compensation[1];

        bool bool_use = true;
        if(fabs(roll_comp) > max_tilt)
            bool_use = false;
        if(fabs(pitch_comp) > max_tilt)
            bool_use = false;
        std::cout << pitch_comp << std::endl;

        if(bool_use)
        {
            // rotation matrix
            Eigen::Matrix3f rot = rotxyz(roll_comp, pitch_comp, 0);
            
            for(int leg_index = 0; leg_index < 4; leg_index++)
            {
                temp.col(leg_index) = rot * temp.col(leg_index); 
            }
        }
        else
        {
            use_imu = false;
        }
    }

    return temp;
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float, 3, 4> RestController::run(State& state, Command& command)
{
    state.foot_locations = step(state, command);
    return state.foot_locations;
}

///////////////////////////////////////////////////////////////////////////////
void RestController::reset_pid_controller()
{
    pid_controller.reset();
}
