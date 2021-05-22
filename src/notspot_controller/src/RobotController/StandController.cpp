/*
 *  StandController.cpp
 *  Author: lnotspotl
 */

#include "notspot_controller/StandController.hpp"
#include "notspot_controller/StateCommand.hpp"

///////////////////////////////////////////////////////////////////////////////
StandController::StandController(Eigen::Matrix<float, 3, 4> default_stance)
{
    def_stance = default_stance;

    FR_X = 0;
    FR_Y = 0;
    FL_X = 0;
    FL_Y = 0;
    max_reach = 0.06;
}

///////////////////////////////////////////////////////////////////////////////
void StandController::updateStateCommand(const sensor_msgs::Joy::ConstPtr& msg,
        State& state, Command& command)
{
    // robot body local position
    state.body_local_position[0] = msg->axes[7] * 0.14; 

    FR_X = msg->axes[1];
    FR_Y = msg->axes[0];
    FL_X = msg->axes[4];
    FL_Y = msg->axes[3];
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float, 3, 4> StandController::default_stance()
{
    return def_stance;
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float, 3, 4> StandController::step(State& state, Command& command)
{
    Eigen::Matrix<float, 3, 4> temp = default_stance();
    for(int i = 0; i < 4; i++)
    {
        temp(2,i) = command.robot_height;
    }

    temp(0,0) += FR_X * max_reach;
    temp(0,1) += FL_X * max_reach;

    temp(1,0) += FR_Y * max_reach;
    temp(1,1) += FL_Y * max_reach;

    return temp;
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float, 3, 4> StandController::run(State& state, Command& command)
{
    state.foot_locations = step(state, command);
    return state.foot_locations;
}
