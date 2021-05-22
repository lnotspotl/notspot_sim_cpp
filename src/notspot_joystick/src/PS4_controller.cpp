/*
 *  PS4Controller.cpp
 *  Author: lnotspotl
 */

#include "notspot_joystick/PS4_controller.hpp"
#include <cmath>

///////////////////////////////////////////////////////////////////////////////
PS4_controller::PS4_controller()
{
    // target
    target_joy.axes = {0.,0.,1.,0.,0.,1.,0.,0.};
    target_joy.buttons = {0,0,0,0,0,0,0,0,0,0,0};

    // last
    last_joy.axes = {0.,0.,1.,0.,0.,1.,0.,0.};
    last_joy.buttons = {0,0,0,0,0,0,0,0,0,0,0};

    last_send_time = ros::Time::now();

    // initialize speed list
    float speeds[] = {0.5, 1.5, 3.0};
    for(int i = 0; i < (sizeof(speeds) / sizeof(speeds[0])); i++)
    {
        speed[i] = speeds[i];
    }
    
    speed_index = 2;
    use_button = true;
}

///////////////////////////////////////////////////////////////////////////////
void PS4_controller::callback(const sensor_msgs::Joy::ConstPtr& msg)
{
    target_joy.axes = msg->axes;
    target_joy.buttons = msg->buttons;

    if(use_button)
    {
        if(target_joy.buttons[4])
        {
            speed_index -= 1;
            if(speed_index < 0)
                speed_index = sizeof(speed) / sizeof(speed[0]) - 1;
            use_button = false;
            ROS_INFO("Joystick speed: %f\n", speed[speed_index]);
        }
        else if(target_joy.buttons[5])
        {
            speed_index += 1;
            if(speed_index >= (sizeof(speed) / sizeof(speed[0])))
                speed_index = 0;
            use_button = false;
            ROS_INFO("Joystick speed: %f\n", speed[speed_index]);
        }
    }

    if(!use_button)
    {
        if(!(target_joy.buttons[4] || target_joy.buttons[5]))
        {
            use_button = true;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
float PS4_controller::ramped_vel(float v_prev, float v_target,
        ros::Time t_prev, ros::Time t_now)
{
    double step = (t_now - t_prev).toSec();
    float sign = v_target > v_prev ? speed[speed_index] : -speed[speed_index];
    float error = std::abs(v_target - v_prev); 

    if(error < (speed[speed_index] * step))
        return v_target;
    else
        return (v_prev + sign * step);
}

///////////////////////////////////////////////////////////////////////////////
bool PS4_controller::run()
{
    bool buttons_change = true;
    bool axes_change = true;

    ros::Time t_now = ros::Time::now();
    
    // check for button changes
    for(int i = 0; i < target_joy.buttons.size(); i++)
    {
        if(target_joy.buttons[i] != last_joy.buttons[i])
            buttons_change = false;
    }

    // check for axes changes
    for(int i = 0; i < target_joy.axes.size(); i++)
    {
        if(target_joy.axes[i] != last_joy.axes[i])
            axes_change = false;
    }


    if(!(buttons_change && axes_change))
    {
        sensor_msgs::Joy new_message; 
        if(!axes_change)
        {
            for(int i = 0; i < target_joy.axes.size(); i++)
            {
                float new_vel;
                if(last_joy.axes[i] == target_joy.axes[i])
                    new_vel = last_joy.axes[i];
                else
                    new_vel = ramped_vel(last_joy.axes[i],target_joy.axes[i],
                            last_send_time, t_now);

                new_message.axes.push_back(new_vel);
            }
        }
        else
        {
            new_message.axes = target_joy.axes;
        }

        new_message.buttons = target_joy.buttons;
        last_joy = new_message;
    }

    last_send_time = t_now;
    return(!(axes_change && buttons_change));
}

///////////////////////////////////////////////////////////////////////////////
sensor_msgs::Joy PS4_controller::get_message()
{
    return last_joy;
}

