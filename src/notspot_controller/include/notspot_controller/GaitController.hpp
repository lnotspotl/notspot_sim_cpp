/*
 *  GaitController.hpp
 *  Author: lnotspotl
 */

#pragma once
#include <Eigen/Core>
#include <vector>
using std::vector;

class GaitController
{
    protected:
        // how long should a leg be in it's stance phase
        float stance_time;
        
        // how long should a leg be in it's swing phase
        float swing_time;

        // controller time step
        float time_step;

        // gait phase plan
        Eigen::MatrixXf contact_phases;

        // robot's default stance
        Eigen::Matrix<float, 3, 4> def_stance;

        // functions
        // GaitController class constructor - set aforementioned variables
        GaitController(float stance_time, float swing_time,
                float time_step, int cont_phases_len,
                Eigen::Matrix<float, 3, 4> default_stance);

        // return default_stance
        Eigen::Matrix<float, 3, 4> default_stance();

        // set new gait phase plan
        void set_contact_phases(vector<vector<int>> new_contact_phases);

        // return number of ticks in stance phase
        int stance_ticks();

        // return number of ticks in swing phase
        int swing_ticks();

        vector<int> phase_ticks();
        int phase_length();

        // current phase column
        int phase_index(int ticks);

        // where in the column are we currenly in?
        int subphase_ticks(int ticks);

        // current phase column
        vector<int> contacts(int ticks);
};
