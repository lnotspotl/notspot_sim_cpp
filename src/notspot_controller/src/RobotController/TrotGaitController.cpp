/*
 *  TrotGaitController.cpp
 *  Author: lnotspotl
 */

#include "notspot_controller/TrotGaitController.hpp"
#include "notspot_controller/Transformations.hpp"
#include <ros/ros.h>

#define DEF_STANCE_LEN 4        // number of phases
#define MAX_X_VELOCITY 0.024    // maximal x velocity
#define MAX_Y_VELOCITY 0.015    // maximal y velocity
#define MAX_YAW_RATE 0.6        // maximal yaw rate around the z-axis
#define Z_LEG_LIFT 0.07         // swing leg lift
#define Z_ERROR_CONSTANT 0.08    

///////////////////////////////////////////////////////////////////////////////
TrotGaitController::TrotGaitController(Eigen::Matrix<float, 3, 4> default_stance,
        float stance_time, float swing_time, float time_step)
: GaitController(stance_time, swing_time, time_step, DEF_STANCE_LEN,
        default_stance),
  trot_swing_controller(stance_ticks(), swing_ticks(),
          phase_length(), time_step, Z_LEG_LIFT, default_stance),
  trot_stance_controller(stance_ticks(), swing_ticks(), phase_length(),
          time_step, Z_ERROR_CONSTANT),
  pid_controller(0.10, 0.08, 0.0)
{
    vector<vector<int>> contact_phases;
    contact_phases.push_back(vector<int>{1,1,1,0});
    contact_phases.push_back(vector<int>{1,0,1,1});
    contact_phases.push_back(vector<int>{1,0,1,1});
    contact_phases.push_back(vector<int>{1,1,1,0});

    set_contact_phases(contact_phases);

    max_x_velocity   = MAX_X_VELOCITY;
    max_y_velocity   = MAX_Y_VELOCITY;
    max_yaw_rate     = MAX_YAW_RATE;

    use_button = true;
    auto_rest = true;
    use_imu = true;
    trot_needed = false;
}

///////////////////////////////////////////////////////////////////////////////
void TrotGaitController::updateStateCommand(const sensor_msgs::Joy::ConstPtr& msg,
        State& state, Command& command) {
    command.velocity[0] = msg->axes[4] * max_x_velocity;
    command.velocity[1] = msg->axes[3] * max_y_velocity;
    command.yaw_rate = msg->axes[0] * max_yaw_rate;

    if(use_button){
        if(msg->buttons[7]){
            use_imu = !use_imu;
            use_button = false;
            if(use_imu)
                ROS_INFO("Trot Gait Controller - Use roll/pitch compensation"
                        " : " "true");
            else
                ROS_INFO("Trot Gait Controller - Use roll/pitch compensation"
                        " : " "false");

        } else if(msg->buttons[6]){
            auto_rest = !auto_rest;    
            if(!auto_rest)
                trot_needed = true;
            use_button = false;
            if(auto_rest)
                ROS_INFO("Trot Gait Controller - Use auto rest"
                        " : " "true");
            else
                ROS_INFO("Trot Gait Controller - Use auto rest"
                        " : " "false");
        }
    }

    if(!use_button)
    {
        if(!(msg->buttons[6] || msg->buttons[7]))
            use_button = true;
    }
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float, 3, 4> TrotGaitController::step(State& state,
        Command& command){

    // auto_rest
    if(auto_rest){
        bool zero_velx = command.velocity[0] == 0;
        bool zero_vely = command.velocity[1] == 0;
        bool zero_yawr = command.yaw_rate == 0;

        if(zero_velx && zero_vely && zero_yawr){
            if(state.ticks % (2 * phase_length()) == 0)
                trot_needed = false;
        } else{
            trot_needed = true;
        }
    }

    if(trot_needed){
        vector<int> contact_modes = contacts(state.ticks);

        Eigen::Matrix<float, 3, 4> new_foot_locations;
        new_foot_locations.setZero();

        for(int leg_index = 0; leg_index < 4; leg_index++){
            int contact_mode = contact_modes[leg_index];
            Eigen::Vector3f new_location;
            if(contact_mode == 1){
                new_location = trot_stance_controller.next_foot_location(
                        leg_index, state, command);
            } else {
                float swing_proportion = (float)subphase_ticks(state.ticks) / 
                    (float)swing_ticks();
                new_location = trot_swing_controller.next_foot_location(
                        swing_proportion, leg_index, state, command);            
            }
            new_foot_locations.col(leg_index) = new_location;
        }

        // tilt compensation
        if(use_imu){
            Eigen::Vector2f compensation;
            compensation = pid_controller.run(state.imu_roll, state.imu_pitch);
            float roll_comp = -compensation[0];
            float pitch_comp = -compensation[1];

            // rotation matrix
            Eigen::Matrix3f rot = rotxyz(roll_comp, pitch_comp, 0);

            // apply rotation
            for(int leg_index = 0; leg_index < 4; leg_index++){
                new_foot_locations.col(leg_index) = rot *
                    new_foot_locations.col(leg_index);
            }
        }

        state.ticks += 1;
        return new_foot_locations;
    }
    else
    {
        Eigen::Matrix<float, 3, 4> new_foot_locations;
        new_foot_locations = default_stance();
        for(int i = 0; i < 4; i++)
            new_foot_locations(2,i) = command.robot_height;
        return new_foot_locations;
    }
}
///////////////////////////////////////////////////////////////////////////////
void TrotGaitController::reset_pid_controller(){
    pid_controller.reset();
}
///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float, 3, 4> TrotGaitController::run(State& state, Command& command)
{
    state.foot_locations = step(state, command);
    state.robot_height = command.robot_height;
    return state.foot_locations;
}

///////////////////////////////////////////////////////////////////////////////
TrotGaitController::TrotStanceController::TrotStanceController(int stance_ticks,
        int swing_ticks, int phase_length, float time_step, 
        float z_error_constant)
{
    this->stance_ticks = stance_ticks;
    this->swing_ticks = swing_ticks;
    this->phase_length = phase_length;
    this->time_step = time_step;
    this->z_error_constant = z_error_constant;
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Vector3f TrotGaitController::TrotStanceController::
position_delta(int leg_index, State& state,
    Command& command){
    float z = state.foot_locations.col(leg_index)[2];

    float step_dist_x = command.velocity[0] * (float)phase_length /
        (float)swing_ticks;

    float step_dist_y = command.velocity[1] * (float)phase_length /
        (float)swing_ticks;

    Eigen::Vector3f delta_pos;
    delta_pos[0] = -(step_dist_x / 4) / (float)(time_step * stance_ticks);
    delta_pos[1] = -(step_dist_y / 4) / (float)(time_step * stance_ticks);
    delta_pos[2] = (1.0 / z_error_constant) * (state.robot_height - z);

    delta_pos *= time_step;

    return delta_pos;
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix3f TrotGaitController::TrotStanceController::
orientation_delta(Command& command){
    Eigen::Matrix3f delta_orientation;
    delta_orientation = rotz(-command.yaw_rate * time_step);
    return delta_orientation;
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Vector3f TrotGaitController::TrotStanceController::
next_foot_location(int leg_index, State& state,
    Command& command){
    Eigen::Vector3f foot_location = state.foot_locations.col(leg_index);
    Eigen::Vector3f delta_pos = position_delta(leg_index, state, command);
    Eigen::Matrix3f delta_ori = orientation_delta(command);

    Eigen::Vector3f next_foot_location;
    next_foot_location = delta_ori * foot_location + delta_pos;
    return next_foot_location;
}

///////////////////////////////////////////////////////////////////////////////
TrotGaitController::TrotSwingController::TrotSwingController(int stance_ticks,
        int swing_ticks, int phase_length, float time_step, float z_leg_lift,
        Eigen::Matrix<float, 3, 4> default_stance)
{
    this->stance_ticks = stance_ticks;
    this->swing_ticks = swing_ticks;
    this->phase_length = phase_length;
    this->time_step = time_step;
    this->z_leg_lift = z_leg_lift;
    this->default_stance = default_stance;
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Vector3f TrotGaitController::TrotSwingController::
raibert_touchdown_location(int leg_index,
        Command& command){
    Eigen::Vector3f delta_pos;
    delta_pos[0] = command.velocity[0] * time_step * phase_length;
    delta_pos[1] = command.velocity[1] * time_step * phase_length;
    delta_pos[2] = 0.0;

    float theta = stance_ticks * time_step * command.yaw_rate;
    Eigen::Matrix3f rotation = rotz(theta);
    
    Eigen::Vector3f result;
    result = rotation * default_stance.col(leg_index) + delta_pos;
    return result;
}

///////////////////////////////////////////////////////////////////////////////
float TrotGaitController::TrotSwingController::swing_height(float swing_phase){
    float swing_height_;
    if(swing_phase < 0.5){
        swing_height_ = swing_phase * 2 * z_leg_lift;
    } else{
        swing_height_ = z_leg_lift * (1.0 - (swing_phase - 0.5) * 2); 
    }
    return swing_height_;
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Vector3f TrotGaitController::TrotSwingController::
next_foot_location(float swing_proportion,
    int leg_index, State& state, Command& command){
    if(!(swing_proportion >= 0.0 && swing_proportion <= 1.0)){
        exit(EXIT_FAILURE);
    } 

    Eigen::Vector3f foot_location = state.foot_locations.col(leg_index);
    float swing_height = this->swing_height(swing_proportion);
    Eigen::Vector3f touchdown_location = raibert_touchdown_location(leg_index,
            command);
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
