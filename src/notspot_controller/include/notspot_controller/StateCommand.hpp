/*
 *  StateCommand.hpp
 *  Author: lnotspotl
 */

#pragma once
#include <Eigen/Core>

enum BehaviorState
{
    REST = 0,
    TROT = 1,
    CRAWL = 2,
    STAND = 3
};

struct State
{
    // robot's velocity
    float velocity[2];

    // robot's yaw rate
    float yaw_rate;

    // robot's height
    float robot_height;

    // current foot locations in the base_link_world coordinate frame
    Eigen::Matrix<float, 3, 4> foot_locations;

    // position of the base_link coordinate frame in base_link_world
    float body_local_position[3];

    // orientation of the base_link coordinate frame in base_link_world
    float body_local_orientation[3];

    // current body roll
    float imu_roll;

    // current body pitch
    float imu_pitch;

    int ticks;
    BehaviorState behavior_state;

    // Constructor
    State(float default_height)
    : velocity{0.0, 0.0}, body_local_position{0.0, 0.0, 0.0},
      body_local_orientation{0.0, 0.0, 0.0}
    {
        yaw_rate = 0.0;
        robot_height = -default_height;
        foot_locations.setZero();
        imu_roll = 0.0;
        imu_pitch = 0.0;
        ticks = 0;
        behavior_state = REST;
    }
};

struct Command
{
    // commanded velocity
    float velocity[2];

    // commanded yaw rate
    float yaw_rate;

    // commanded robot height
    float robot_height;

    // trot request
    bool trot_event;

    // crawl request
    bool crawl_event;

    // rest request
    bool rest_event;

    // stand request
    bool stand_event;

    // Constructor
    Command(float default_height)
    : velocity{0.0, 0.0}
    {
        yaw_rate = 0.0;
        robot_height = -default_height;

        trot_event = false;
        crawl_event = false;
        rest_event = false;
        stand_event = false;
    }
};
