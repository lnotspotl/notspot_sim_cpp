/*
 *  GaitController.cpp
 *  Author: lnotspotl
 */

#include "notspot_controller/GaitController.hpp"
#include <ros/ros.h>

///////////////////////////////////////////////////////////////////////////////
GaitController::GaitController(float stance_time, float swing_time,
        float time_step, int cont_phases_len,
        Eigen::Matrix<float, 3, 4> default_stance)
:contact_phases(4, cont_phases_len)
{
    this->stance_time = stance_time;
    this->swing_time = swing_time;
    this->time_step = time_step;

    this->def_stance = default_stance;
}

///////////////////////////////////////////////////////////////////////////////
void GaitController::set_contact_phases(vector<vector<int>> new_contact_phases)
{
    int n_cols = contact_phases.cols();
    int n_rows = contact_phases.rows();

    if(new_contact_phases.size() != n_rows)
    {
        ROS_FATAL("GaitController - contact_phases - number of"
                "rows does not match!");
        exit(EXIT_FAILURE);
    }

    if(new_contact_phases[0].size() != n_cols)
    {
        ROS_FATAL("GaitController - contact_phases - number of"
                "columns does not match!");
        exit(EXIT_FAILURE);
    }

    for(int i = 0; i < n_rows; i++)
    {
        for(int j = 0; j < n_cols; j++)
        {
            contact_phases(i,j) = new_contact_phases[i][j];
        }
    }

}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float, 3, 4> GaitController::default_stance()
{
    return def_stance;
}

///////////////////////////////////////////////////////////////////////////////
int GaitController::stance_ticks()
{
    return((int)(stance_time / time_step));
}

///////////////////////////////////////////////////////////////////////////////
int GaitController::swing_ticks()
{
    return((int)(swing_time / time_step));
}

///////////////////////////////////////////////////////////////////////////////
vector<int> GaitController::phase_ticks()
{
    vector<int> temp;
    int n_rows = contact_phases.rows();
    int n_cols = contact_phases.cols();
    for(int i = 0; i < n_cols; i++)
    {
        bool contains_zero = false;
        for(int k = 0; k < n_rows; k++)
        {
            if(contact_phases(k,i) == 0)
            {
                contains_zero = true;
                break;
            }
        }

        if(contains_zero)
        {
            temp.push_back(swing_ticks());
        }
        else
        {
            temp.push_back(stance_ticks());
        }
    }
    return temp;
}

///////////////////////////////////////////////////////////////////////////////
int GaitController::phase_length()
{
    vector<int> phase_ticks = this->phase_ticks();
    int sum = 0;
    for(int i = 0; i < phase_ticks.size(); i++)
    {
        sum += phase_ticks[i]; 
    }
    return sum;
}

///////////////////////////////////////////////////////////////////////////////
int GaitController::phase_index(int ticks)
{
    float phase_time = ticks % phase_length();
    int phase_sum = 0;
    int n_cols = contact_phases.cols();
    vector<int> phase_ticks = this->phase_ticks();
    for(int i = 0; i < n_cols; i++)
    {
        phase_sum += phase_ticks[i];
        if(phase_time < phase_sum)
        {
            return i;
        }
    }

    // Some error has occured
    exit(EXIT_FAILURE);
}

///////////////////////////////////////////////////////////////////////////////
int GaitController::subphase_ticks(int ticks)
{
    float phase_time = ticks % phase_length();
    int phase_sum = 0;
    int n_cols = contact_phases.cols();
    vector<int> phase_ticks = this->phase_ticks();
    for(int i = 0; i < n_cols; i++)
    {
        phase_sum += phase_ticks[i];
        if(phase_time < phase_sum)
        {
            float subphase_ticks = phase_time - phase_sum + phase_ticks[i];
            return subphase_ticks;
        }
    }

    // Some error has occured
    exit(EXIT_FAILURE);
}

///////////////////////////////////////////////////////////////////////////////
vector<int> GaitController::contacts(int ticks)
{
    int contact_phase_column = phase_index(ticks);
    vector<int> contact_phases;
    for(int i = 0; i < 4; i++)
    {
        contact_phases.push_back(this->contact_phases(i, contact_phase_column));
    }
    return contact_phases;
}
