/*
 *  TrotGaitController.hpp
 *  Author: lnotspotl
 */

#pragma once
#include <Eigen/Core>
#include <sensor_msgs/Joy.h>
#include <vector>
using std::vector;

#include "notspot_controller/GaitController.hpp"
#include "notspot_controller/StateCommand.hpp"
#include "notspot_controller/PIDController.hpp"

class TrotGaitController : public GaitController
{
    public:

        // TrotGaitController class constructor - set default stance,
        // stance_time, swing_time and time_step
        TrotGaitController(Eigen::Matrix<float, 3, 4> default_stance,
                float stance_time, float swing_time, float time_step);

		// ROS joystick callback - update state and other variables
        void updateStateCommand(const sensor_msgs::Joy::ConstPtr& msg,
                State& state, Command& command);

        // Main run function, return leg positions in the base_link_world frame
        Eigen::Matrix<float, 3, 4> run(State& state, Command& command);

        // reset pid controller
        void reset_pid_controller();

    private:

        //--------------------------------------------------------------------
        class TrotSwingController
        {
            public:
                TrotSwingController(int stance_ticks, int swing_ticks,
                        int phase_lenght, float time_step, float z_leg_lift,
                        Eigen::Matrix<float, 3, 4> default_stance);
                Eigen::Vector3f next_foot_location(float swing_proportion,
                        int leg_index, State& state, Command& command);

            private:
                // variables
                int stance_ticks;
                int swing_ticks;
                int phase_length;
                float time_step;
                float z_leg_lift;
                Eigen::Matrix<float, 3, 4> default_stance;

                // functions
                Eigen::Vector3f raibert_touchdown_location(int leg_index,
                        Command& command); 
                float swing_height(float swing_phase);

        };

        //--------------------------------------------------------------------
        class TrotStanceController
        {
            public:
                TrotStanceController(int stance_ticks, int swing_ticks,
                        int phase_length, float time_step,
                        float z_error_constant);
                Eigen::Vector3f next_foot_location(int leg_index, State& state,
                        Command& command);

            private:
                int stance_ticks;
                int swing_ticks;
                int phase_length;
                float time_step;
                float z_error_constant;
                Eigen::Vector3f position_delta(int leg_index, State& state,
                        Command& command);
                Eigen::Matrix3f orientation_delta(Command& command);
        };
        //--------------------------------------------------------------------
        
        // determines, how fast the leg moves towards the goal in the
        // z direction
        float z_error_constant;

        // maximal leg lift
        float z_leg_lift;

        // maximal velocity in the x direction
        float max_x_velocity;

        // maximal velocity in the y direction
        float max_y_velocity;

        // maximal yaw rate around the z axis
        float max_yaw_rate;

        // joystick anti-spam bool 
        bool use_button;

        // should the robot try to automatically rest if it does not move anywhere
        bool auto_rest;

        // shoudl the robot use tilt compensation
        bool use_imu;

        // robot needs move
        bool trot_needed;

        // swing controller
        TrotSwingController trot_swing_controller;

        // stance controller 
        TrotStanceController trot_stance_controller;

        // pid controller - used for tilt compensation
        PIDController pid_controller;

        // Controller step - return new leg positions
        Eigen::Matrix<float, 3, 4> step(State& state, Command& command);
};
