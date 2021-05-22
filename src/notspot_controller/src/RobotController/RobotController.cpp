/*
 *  RobotController.cpp
 *  Author: lnotspotl
 */

#include "notspot_controller/RobotController.hpp"
#include "notspot_controller/StateCommand.hpp"
#include <tf/transform_datatypes.h>

#define ROBOT_HEIGHT 0.15
#define X_SHIFT_FRONT 0.007
#define X_SHIFT_BACK -0.03

///////////////////////////////////////////////////////////////////////////////
RobotController::RobotController(const float body[], const float legs[])
: state(ROBOT_HEIGHT), command(ROBOT_HEIGHT),
  delta_x(body[0] * 0.5), delta_y(body[1]*0.5 + legs[1]),
  x_shift_front(X_SHIFT_FRONT), x_shift_back(X_SHIFT_BACK),
  restController(default_stance()),
  trotGaitController(default_stance(),0.18, 0.24, 0.02),
  crawlGaitController(default_stance(), 0.50, 0.40, 0.02),
  standController(default_stance())
{
    // body dimensions
    this->body[0] = body[0];
    this->body[1] = body[1];

    // leg dimensions
    this->legs[0] = legs[0];

    this->legs[1] = legs[1];
    this->legs[2] = legs[2];
    this->legs[3] = legs[3];

    state.foot_locations = default_stance();
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float, 3, 4> RobotController::default_stance()
{
    // FR, FL, RR, RL
    Eigen::Matrix<float, 3, 4> default_coordinates;
    default_coordinates <<   delta_x + x_shift_front, // FR - x
                             delta_x + x_shift_front, // FL - x
                            -delta_x + x_shift_back,  // RR - x
                            -delta_x + x_shift_back,  // RL - x

                            -delta_y,   // FR - y
                             delta_y,   // FL - y
                            -delta_y,   // RR - y
                             delta_y,   // RL - y

                             0,     // FR - z
                             0,     // FL - z
                             0,     // RR - z
                             0;     // RL - z

    return default_coordinates;
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float, 3, 4> RobotController::run()
{
    if(state.behavior_state == REST)
    {
        return(restController.run(state,command));
    }
    else if(state.behavior_state == TROT)
    {
        return(trotGaitController.run(state, command)); 
    }
    else if(state.behavior_state == CRAWL)
    {
        return(crawlGaitController.run(state, command)); 
    }
    else if(state.behavior_state == STAND)
    {
        return(standController.run(state, command)); 
    }
    else
    {
        exit(EXIT_FAILURE);
    }
}

///////////////////////////////////////////////////////////////////////////////
void RobotController::joystick_command(const sensor_msgs::Joy::ConstPtr& msg)
{
    if(msg->buttons[0]) { // rest
        command.trot_event = false;
        command.crawl_event = false;
        command.stand_event = false;
        command.rest_event = true;
    } else if (msg->buttons[1]) { // trot
        command.trot_event = true;
        command.crawl_event = false;
        command.stand_event = false;
        command.rest_event = false;
    } else if(msg->buttons[2]) { // crawl
        command.trot_event = false;
        command.crawl_event = true;
        command.stand_event = false;
        command.rest_event = false;
    } else if(msg->buttons[3]) { // stand
        command.trot_event = false;
        command.crawl_event = false;
        command.stand_event = true;
        command.rest_event = false;
    }

    if(state.behavior_state == REST) {
        restController.updateStateCommand(msg, state, command);
    } else if(state.behavior_state == TROT) {
        trotGaitController.updateStateCommand(msg, state, command);
    } else if(state.behavior_state == CRAWL) {
        crawlGaitController.updateStateCommand(msg, state, command); 
    } else if(state.behavior_state == STAND) {
        standController.updateStateCommand(msg, state, command); 
    } else {
        exit(EXIT_FAILURE);
    }
}

///////////////////////////////////////////////////////////////////////////////
void RobotController::change_controller(){
    if(command.trot_event){
        if(state.behavior_state == REST){
            state.behavior_state = TROT;
            state.ticks = 0;
            trotGaitController.reset_pid_controller();
        }
        command.trot_event = false;
    } else if(command.crawl_event){
        if(state.behavior_state == REST){
            state.behavior_state = CRAWL;
            crawlGaitController.first_cycle = true;
            state.ticks = 0;
        } 
        command.crawl_event = false;
    } else if(command.stand_event){
        state.behavior_state = STAND;
        command.stand_event = false;
    } else if(command.rest_event){
        state.behavior_state = REST;
        command.rest_event = false;
    }
}

///////////////////////////////////////////////////////////////////////////////
void RobotController::imu_orientation(const sensor_msgs::Imu::ConstPtr& msg){
    float x = msg->orientation.x;
    float y = msg->orientation.y;
    float z = msg->orientation.z;
    float w = msg->orientation.w;

    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    state.imu_roll = roll;
    state.imu_pitch = pitch;
}
