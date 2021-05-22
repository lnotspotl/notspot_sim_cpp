/*
 *  PS4Controller.hpp
 *  Author: lnotspotl
 */

#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

class PS4_controller
{
    private:
        // target joystick state
        sensor_msgs::Joy target_joy;

        // last joystick state
        sensor_msgs::Joy last_joy;

        // time, when we last published a message with joystick state
        ros::Time last_send_time;

        // ramped velocity
        float ramped_vel(float v_prev, float v_target,
                ros::Time t_prev, ros::Time t_now);

        // different ramped velocity speeds
        float speed[3];

        // index of the current speed
        int speed_index;

        // anti-spam bool
        bool use_button;

    public:
        // constructor
        PS4_controller();

        // ROS joystick callback -> subscribes to the /joy topic
        void callback(const sensor_msgs::Joy::ConstPtr& msg); 

        // run controller
        bool run();

        // get a message that is to be published
        sensor_msgs::Joy get_message();
};
