/*
 *  Transformations.cpp
 *  Author: lnotspotl
 */

#include <Eigen/Geometry>
#include "notspot_controller/Transformations.hpp"

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix3f rotx(float alpha)
{
    Eigen::Matrix3f x_rotation_matrix;
    x_rotation_matrix = Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitX());
    return x_rotation_matrix;
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix3f roty(float beta)
{
    Eigen::Matrix3f y_rotation_matrix;
    y_rotation_matrix = Eigen::AngleAxisf(beta, Eigen::Vector3f::UnitY());
    return y_rotation_matrix;
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix3f rotz(float gamma)
{
    Eigen::Matrix3f z_rotation_matrix;
    z_rotation_matrix = Eigen::AngleAxisf(gamma, Eigen::Vector3f::UnitZ());
    return z_rotation_matrix;
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix3f rotxyz(float alpha, float beta, float gamma)
{
    Eigen::Matrix3f xyz_rotation_matrix;
    xyz_rotation_matrix = rotx(alpha) * roty(beta) * rotz(gamma);
    return xyz_rotation_matrix;
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4f homog_transxyz(float dx, float dy, float dz)
{
    Eigen::Matrix4f translation_matrix = Eigen::Matrix<float, 4, 4>::Identity();
    translation_matrix(0,3) = dx;
    translation_matrix(1,3) = dy;
    translation_matrix(2,3) = dz;
    return translation_matrix;
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4f homog_transform(float dx, float dy, float dz, 
        float alpha, float beta, float gamma)
{
    Eigen::Matrix4f transformation_matrix = homog_transxyz(dx, dy, dz);
    transformation_matrix.block<3,3>(0,0) = rotxyz(alpha, beta, gamma);
    return transformation_matrix;
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4f homog_transform_inverse(Eigen::Matrix4f matrix)
{
    Eigen::Matrix4f inverse_matrix = Eigen::Matrix<float, 4, 4>::Identity();
    inverse_matrix.block(0,0,3,3) = matrix.block(0,0,3,3).transpose();
    inverse_matrix.block(0,3,3,1) = -1 * inverse_matrix.block(0,0,3,3) *
        matrix.block(0,3,3,1);
    return inverse_matrix;
}
