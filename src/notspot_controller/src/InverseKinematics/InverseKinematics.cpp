/*
 *  InverseKinematics.cpp
 *  Author: lnotspotl
 */

#include <vector>
#include <math.h>
#include <ros/ros.h>

#include "notspot_controller/InverseKinematics.hpp"
#include "notspot_controller/Transformations.hpp"

///////////////////////////////////////////////////////////////////////////////
InverseKinematics::InverseKinematics(const float body_dimensions[],
        const float leg_dimensions[])
{
    // body dimensions
    body_length = body_dimensions[0];
    body_width = body_dimensions[1];

    // leg dimensions
    a1 = leg_dimensions[0];
    d2 = leg_dimensions[1];
    a3 = leg_dimensions[2];
    a4 = leg_dimensions[3];
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float, 3, 4> InverseKinematics::get_local_positions(
        Eigen::Matrix<float, 3, 4> leg_positions, float dx, float dy,
        float dz, float roll, float pitch, float yaw)
{
    // make a 4x4 leg position matrix (4 x leg position vector)
    Eigen::Matrix4f leg_positions_matrix;
    leg_positions_matrix.block<3,4>(0,0) = leg_positions;
    for(int i = 0; i < 4; i++)
    {
        leg_positions_matrix(3,i) = 1.0;
    }

    // Transformations matrix, base_link_world => base_link
    Eigen::Matrix4f T_blwbl = homog_transform(dx, dy, dz, roll, pitch, yaw);

    // Transformation matrix, base_link_world => FR1
    Eigen::Matrix4f T_blwFR1 = T_blwbl * homog_transform(
        0.5 * body_length, -0.5 * body_width, 0.0,
        PI/2, -PI/2, 0.0);
    
    // Transformation matrix, base_link_world => FL1
    Eigen::Matrix4f T_blwFL1 = T_blwbl * homog_transform(
        0.5 * body_length, 0.5 * body_width, 0.0,
        PI/2, -PI/2, 0.0);

    // Transformation matrix, base_link_world => RR1
    Eigen::Matrix4f T_blwRR1 = T_blwbl * homog_transform(
        -0.5 * body_length, -0.5 * body_width, 0.0,
        PI/2, -PI/2, 0.0);
    
    // Transformation matrix, base_link_world => RL1
    Eigen::Matrix4f T_blwRL1 = T_blwbl * homog_transform(
        -0.5 * body_length, 0.5 * body_width, 0.0,
        PI/2, -PI/2, 0.0);

    // apply transformations
    // FR
    leg_positions_matrix.col(0) = homog_transform_inverse(T_blwFR1) *
                leg_positions_matrix.col(0);

    // FL
    leg_positions_matrix.col(1) = homog_transform_inverse(T_blwFL1) *
                leg_positions_matrix.col(1);
    
    // RR
    leg_positions_matrix.col(2) = homog_transform_inverse(T_blwRR1) *
                leg_positions_matrix.col(2);

    // RL
    leg_positions_matrix.col(3) = homog_transform_inverse(T_blwRL1) *
                leg_positions_matrix.col(3);

    return leg_positions_matrix.block<3,4>(0,0); 
}

///////////////////////////////////////////////////////////////////////////////
std::vector<double> InverseKinematics::inverse_kinematics(
    Eigen::Matrix<float, 3, 4> leg_positions, float dx, float dy,
    float dz, float roll, float pitch, float yaw)
{
    Eigen::Matrix<float, 3, 4> positions = get_local_positions(
        leg_positions, dx, dy, dz, roll, pitch, yaw);

    std::vector<double> angles;

    for(int i = 0; i < 4; i++)
    {
        double x = positions.col(i)[0];
        double y = positions.col(i)[1];
        double z = positions.col(i)[2];

        double F = sqrt(x*x + y*y - d2*d2);
        double G = F - a1;
        double H = sqrt(G*G + z*z);

        double theta1 = atan2(y, x) + atan2(F, d2 * pow((-1), i));

        double D = (H*H - a3*a3 - a4*a4) / (2*a3*a4);
        
        double theta4 = -atan2((sqrt(1-D*D)), D);

        double theta3 = atan2(z,G) - atan2(a4 * sin(theta4),
                a3 + a4 * cos(theta4));

        angles.push_back(theta1);
        angles.push_back(theta3);
        angles.push_back(theta4);
    }

    return angles;
}
