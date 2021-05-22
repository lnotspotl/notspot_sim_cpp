/*
 *  CrawlGaitController.hpp
 *  Author: lnotspotl
 */

#include <ros/ros.h>

#include "notspot_controller/CrawlGaitController.hpp"
#include "notspot_controller/Transformations.hpp"

#define DEF_STANCE_LEN 8
#define MAX_X_VELOCITY 0.003
#define MAX_YAW_RATE 0.3
#define Z_LEG_LIFT 0.07
#define Z_ERROR_CONSTANT 0.08
#define BODY_SHIFT_Y 0.030

///////////////////////////////////////////////////////////////////////////////
CrawlGaitController::CrawlGaitController(
    Eigen::Matrix<float, 3, 4> default_stance, float stance_time,
    float swing_time, float time_step)
: GaitController(stance_time, swing_time, time_step, DEF_STANCE_LEN,
        default_stance),
  crawl_swing_controller(stance_ticks(), swing_ticks(),
          time_step, phase_length(), Z_LEG_LIFT, default_stance,
          BODY_SHIFT_Y),
  crawl_stance_controller(phase_length(), stance_ticks(), swing_ticks(),
          time_step, Z_ERROR_CONSTANT, BODY_SHIFT_Y)
{
    vector<vector<int>> contact_phases;
    contact_phases.push_back(vector<int>{1,1,1,0,1,1,1,1});
    contact_phases.push_back(vector<int>{1,1,1,1,1,1,1,0});
    contact_phases.push_back(vector<int>{1,0,1,1,1,1,1,1});
    contact_phases.push_back(vector<int>{1,1,1,1,1,0,1,1});

    set_contact_phases(contact_phases);

    max_x_velocity = MAX_X_VELOCITY;
    max_yaw_rate   = MAX_YAW_RATE;

    first_cycle    = true;
}

///////////////////////////////////////////////////////////////////////////////
void CrawlGaitController::updateStateCommand(const sensor_msgs::Joy::ConstPtr&
        msg, State& state, Command& command){
    command.velocity[0] = msg->axes[4] * max_x_velocity;
    command.yaw_rate = msg->axes[0] * max_yaw_rate;
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float, 3, 4> CrawlGaitController::run(State& state,
        Command& command){
    state.foot_locations = step(state, command);
    state.robot_height = command.robot_height;
    
    if(phase_index(state.ticks) > 0 && first_cycle)
        first_cycle = false;

    return state.foot_locations;
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float, 3, 4> CrawlGaitController::step(State& state,
        Command& command){
    vector<int> contact_modes = contacts(state.ticks);
    
    Eigen::Matrix<float, 3, 4> new_foot_locations;
    new_foot_locations.setZero();

    int phase_index = this->phase_index(state.ticks);

    for(int leg_index = 0; leg_index < 4; leg_index++){
        int contact_mode = contact_modes[leg_index];
        Eigen::Vector3f new_location;
        if(contact_mode == 1){
            bool move_sideways = true;
            bool move_left = false;
            if(phase_index == 0 || phase_index == 4){
                if(phase_index == 0)
                    move_left = true;
            } else {
                move_sideways = false;
            }

            new_location = crawl_stance_controller.next_foot_location(leg_index,
                    state, command, first_cycle, move_sideways, move_left);
        } else {
            float swing_proportion = (float)subphase_ticks(state.ticks) /
                (float)swing_ticks();
            bool shifted_left = false;
            if(phase_index == 1 || phase_index == 3)
                shifted_left = true;

            new_location = crawl_swing_controller.next_foot_location(
                    swing_proportion, leg_index, state, command, shifted_left);
        }
        new_foot_locations.col(leg_index) = new_location;
    }

    state.ticks += 1;
    return new_foot_locations;
}

///////////////////////////////////////////////////////////////////////////////
CrawlGaitController::CrawlSwingController::CrawlSwingController(int stance_ticks,
        int swing_ticks, float time_step, int phase_length, float z_leg_lift,
        Eigen::Matrix<float, 3, 4> default_stance, float body_shift_y)
{
    this->stance_ticks = stance_ticks;
    this->swing_ticks = swing_ticks;
    this->time_step = time_step;
    this->phase_length = phase_length;
    this->z_leg_lift = z_leg_lift;
    this->default_stance = default_stance;
    this->body_shift_y = body_shift_y;
} 

///////////////////////////////////////////////////////////////////////////////
Eigen::Vector3f CrawlGaitController::CrawlSwingController::next_foot_location(
    float swing_proportion, int leg_index, State& state, Command& command,
    bool shifted_left){
    if(!(swing_proportion >= 0.0 && swing_proportion <= 1.0)){
        exit(EXIT_FAILURE);
    } 

    Eigen::Vector3f foot_location = state.foot_locations.col(leg_index);
    float swing_height = this->swing_height(swing_proportion);
    Eigen::Vector3f touchdown_location = raibert_touchdown_location(leg_index,
            command, shifted_left);
    float time_left = time_step * swing_ticks * (1.0 - swing_proportion);

    Eigen::Vector3f delta_foot_location;
    delta_foot_location[0] = (touchdown_location[0] - foot_location[0]) / time_left;
    delta_foot_location[1] = (touchdown_location[1] - foot_location[1]) / time_left;

    delta_foot_location[0] = delta_foot_location[0] * time_step;
    delta_foot_location[1] = delta_foot_location[1] * time_step;
    delta_foot_location[2] = swing_height + command.robot_height;

    foot_location[0] += delta_foot_location[0];
    foot_location[1] += delta_foot_location[1];
    foot_location[2] = delta_foot_location[2];

    return foot_location;
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Vector3f CrawlGaitController::
CrawlSwingController::raibert_touchdown_location(int leg_index,
    Command& command, bool shifted_left){
    Eigen::Vector3f delta_pos;
    delta_pos[0] = command.velocity[0] * time_step * phase_length;
    delta_pos[1] = 0.0;
    delta_pos[2] = 0.0;

    float shift_correction = body_shift_y;
    if(shifted_left){
        shift_correction = -body_shift_y;
    }

    float theta = stance_ticks * time_step * command.yaw_rate;
    Eigen::Matrix3f rotation = rotz(theta);

    Eigen::Vector3f result;
    result = rotation * default_stance.col(leg_index) + delta_pos;
    result[1] += shift_correction;
    return result;
}

///////////////////////////////////////////////////////////////////////////////
float CrawlGaitController::CrawlSwingController::swing_height(float swing_phase){
    float swing_height_;
    if(swing_phase < 0.5){
        swing_height_ = swing_phase * 2 * z_leg_lift;
    } else{
        swing_height_ = z_leg_lift * (1.0 - (swing_phase - 0.5) * 2); 
    }
    return swing_height_;
}

///////////////////////////////////////////////////////////////////////////////
CrawlGaitController::CrawlStanceController::CrawlStanceController(
    int phase_length, int stance_ticks, int swing_ticks, float time_step,
    float z_error_constant, float body_shift_y)
{
    this->phase_length = phase_length;
    this->stance_ticks = stance_ticks;
    this->swing_ticks = swing_ticks;
    this->time_step = time_step;
    this->z_error_constant = z_error_constant;
    this->body_shift_y = body_shift_y;
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Vector3f CrawlGaitController::
CrawlStanceController::position_delta(int leg_index, State& state,
    Command& command, bool first_cycle, bool move_sideways, bool move_left){
    float z = state.foot_locations.col(leg_index)[2];

    float step_dist_x = command.velocity[0] * (float)phase_length / 
        (float)swing_ticks;

    int shift_factor = 2;
    if(first_cycle)
        shift_factor = 1;

    float side_vel = 0.0;
    if(move_sideways){
        if(move_left){
            side_vel = -(body_shift_y * shift_factor) / (float)(time_step * 
                    stance_ticks);
        } else {
            side_vel = (body_shift_y * shift_factor) / (float)(time_step * 
                    stance_ticks);
        }
    }

    Eigen::Vector3f delta_pos;
    delta_pos[0] = -(step_dist_x / 3) / (float)(time_step * stance_ticks);
    delta_pos[1] = side_vel;
    delta_pos[2] = 1.0 / z_error_constant * (state.robot_height - z);

    delta_pos *= time_step;

    return delta_pos;
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix3f CrawlGaitController::
CrawlStanceController::orientation_delta(Command& command){
    Eigen::Matrix3f delta_orientation;
    delta_orientation = rotz(-command.yaw_rate * time_step);
    return delta_orientation;
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Vector3f CrawlGaitController::
CrawlStanceController::next_foot_location(int leg_index, State& state,
    Command& command, bool first_cycle, bool move_sideways, bool move_left){
    Eigen::Vector3f foot_location = state.foot_locations.col(leg_index);
    Eigen::Vector3f delta_pos = position_delta(leg_index, state, command, 
            first_cycle, move_sideways, move_left);
    Eigen::Matrix3f delta_ori = orientation_delta(command);

    Eigen::Vector3f next_foot_location;
    next_foot_location = delta_ori * foot_location + delta_pos;
    return next_foot_location;
}


