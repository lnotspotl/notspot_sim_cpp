/*
 *  CrawlGaitController.hpp
 *  Author: lnotspotl
 */

#pragma once
#include <Eigen/Core>
#include <sensor_msgs/Joy.h>
#include <vector>
using std::vector;

#include "notspot_controller/GaitController.hpp"
#include "notspot_controller/StateCommand.hpp"

class CrawlGaitController : public GaitController {
    public:

        // CrawlGaitController class constructor - set default stance,
        // stance_time, swing_time and time_step
        CrawlGaitController(Eigen::Matrix<float, 3, 4> default_stance,
                float stance_time, float swing_time, float time_step);

		// ROS joystick callback - update state and other variables
        void updateStateCommand(const sensor_msgs::Joy::ConstPtr& msg,
                State& state, Command& command);

        // Main run function, return leg positions in the base_link_world frame
        Eigen::Matrix<float, 3, 4> run(State& state, Command& command);

        bool first_cycle;
        
    private:

        //--------------------------------------------------------------------
        class CrawlSwingController{
            public:
                CrawlSwingController(int stance_ticks, int swing_ticks,
                        float time_step, int phase_length, float z_leg_lift,
                        Eigen::Matrix<float, 3, 4> default_stance,
                        float body_shift_y);

                Eigen::Vector3f next_foot_location(float swing_proportion,
                        int leg_index, State& state, Command& command,
                        bool shifted_left);

            private:
                int stance_ticks;
                int swing_ticks;
                float time_step;
                int phase_length;
                float z_leg_lift;
                Eigen::Matrix<float, 3, 4> default_stance;
                float body_shift_y;

                Eigen::Vector3f raibert_touchdown_location(int leg_index,
                        Command& command, bool shifted_left); 
                float swing_height(float swing_phase);
        };
        //--------------------------------------------------------------------
        class CrawlStanceController{
            public:
                CrawlStanceController(int phase_length, int stance_ticks,
                        int swing_ticks, float time_step, float z_error_constant,
                        float body_shift_y);

                Eigen::Vector3f next_foot_location(int leg_index, State& state,
                        Command& command, bool first_cycle, bool move_sideways,
                        bool move_left);
            private:
                int phase_length;
                int stance_ticks;
                int swing_ticks;
                float time_step;
                float z_error_constant;
                float body_shift_y;

                Eigen::Vector3f position_delta(int leg_index, State& state,
                        Command& command, bool first_cycle, bool move_sideways,
                        bool move_left);
                Eigen::Matrix3f orientation_delta(Command& command);
        };
        //--------------------------------------------------------------------

        // maximal velocity in the x direction
        float max_x_velocity;

        // maximal yaw rate around the z axis
        float max_yaw_rate;

        // distance to move the body to the side in order for the robot
        // not to tip over
        float body_shift_y;

        // stance controller
        CrawlStanceController crawl_stance_controller;

        // swing controller
        CrawlSwingController crawl_swing_controller;

        // Controller step - return new leg positions
        Eigen::Matrix<float, 3, 4> step(State& state, Command& command);
};

