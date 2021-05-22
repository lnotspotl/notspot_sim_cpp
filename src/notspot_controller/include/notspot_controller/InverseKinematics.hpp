/*
 *  InverseKinematics.hpp
 *  Author: lnotspotl
 */

#pragma once
#include <Eigen/Core>
#include <vector>
using std::vector;

class InverseKinematics
{
    private:
        // robot's body length
        float body_length;

        // robot's body width
        float body_width;

        // leg's a1 dimension
        float a1;

        // leg's d2 dimension
        float d2;

        // leg's a3 dimension
        float a3;
        
        // leg's a4 dimension
        float a4;

        // Compute legs' endpoint coordinates in their associated shoulder frames.
        Eigen::Matrix<float, 3, 4> get_local_positions(
                Eigen::Matrix<float, 3, 4> leg_positions, float dx, float dy,
                float dz, float roll, float pitch, float yaw);

    public:
        // InverseKinematics class constructor - set body and leg dimensions
        InverseKinematics(const float body_dimensions[],
                const float leg_dimensions[]);

        // Compute inverse kinematics for given leg positions
        vector<double> inverse_kinematics(
                Eigen::Matrix<float, 3, 4> leg_positions, float dx, float dy,
                float dz, float roll, float pitch, float yaw);
};
