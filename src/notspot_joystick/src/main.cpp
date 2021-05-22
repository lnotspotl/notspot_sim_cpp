/*
 *  main.cpp
 *  Author: lnotspotl
 */

#include "notspot_joystick/PS4_controller.hpp"
#include <ros/ros.h>

#define RATE 30

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "joystick_ramped"); 
    ros::NodeHandle node_handle;

    PS4_controller joystick;

    // /joy subscriber
    ros::Subscriber joy_sub = node_handle.subscribe("joy", 1,
            &PS4_controller::callback, &joystick);

    // /notspot_joy/joy_ramped publisher
    ros::Publisher joy_ramped_pub = node_handle.advertise<sensor_msgs::Joy>
            ("notspot_joy/joy_ramped", 1);

    ros::Rate loop_rate(RATE);
    while(ros::ok())
    {
        bool should_publish = joystick.run();
        if(should_publish)
            joy_ramped_pub.publish(joystick.get_message());
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
