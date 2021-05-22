/*
 *  Transformations.hpp
 *  Author: lnotspotl
 */

#pragma once
#include <Eigen/Core>

#define PI 3.1415926535897

// Rotation around the x axis
Eigen::Matrix3f rotx(float alpha);

// Rotation around the y axis
Eigen::Matrix3f roty(float beta);

// Rotation around the z axis
Eigen::Matrix3f rotz(float gamma);

// Rotation around the x axis -> y axis -> z axis
Eigen::Matrix3f rotxyz(float alpha, float beta, float gamma);

// Transformation along the x, y and z axis
Eigen::Matrix4f homog_transxyz(float dx, float dy, float dz);

// 4x4 general transformation matrix
Eigen::Matrix4f homog_transform(float dx, float dy, float dz, 
        float alpha, float beta, float gamma);

// Inverse of a general 4x4 transformation matrix
Eigen::Matrix4f homog_transform_inverse(Eigen::Matrix4f matrix);
