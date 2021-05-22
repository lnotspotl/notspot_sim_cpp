/*
 *  PIDController.hpp
 *  Author: lnotspotl
 */

#pragma once
#include <Eigen/Core>
#include <ros/ros.h>

class PIDController
{
    public:
        // PIDController class constructor - p, i and d values
        PIDController(float p, float i, float d);

		// Set desired roll and pitch angles
        void set_desired_RP_angles(float des_roll, float des_pitch);

		// Reset PID controller
        void reset();

		// Run 
        Eigen::Vector2f run(float roll, float pitch);

    private:
		// p value
        float kp;

		// i value
        float ki;

		// d value
        float kd;


		// desired roll and pitch angles
        Eigen::Vector2f desired_roll_pitch;

		// accumulated I term
        Eigen::Vector2f I_term;

		// last error
        Eigen::Vector2f last_error;

        // anti-windup -> I_term cannot be more than max_I or less then -max_I
        float max_I;

		// last call of the run function
        ros::Time last_send_time;
};
