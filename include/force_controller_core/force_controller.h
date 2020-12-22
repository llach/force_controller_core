/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Luca Lach
*/

#ifndef FORCE_CONTROLLER_CORE_FORCE_CONTROLLER_H
#define FORCE_CONTROLLER_CORE_FORCE_CONTROLLER_H

#include <map>
#include <deque>
#include <memory>
#include <vector>
#include <numeric>

namespace fcc {

// controller states
enum CONTROLLER_STATE {TRAJECTORY_EXEC, TRANSITION, FORCE_CTRL};

// sensor/joint states
enum SENSOR_STATE {NO_CONTACT, LOST_CONTACT, GOT_CONTACT, IN_CONTACT, GOAL, VIOLATED};

std::map<SENSOR_STATE, std::string> STATE_STRING = {
    {NO_CONTACT, "no contact"},
    {GOT_CONTACT, "got contact"},
    {LOST_CONTACT, "lost contact"},
    {IN_CONTACT, "still in contact"},
    {GOAL, "reached goal"},
    {VIOLATED, "violated goal constraints"}
};

class JointForceController
{
public:
    JointForceController(
            std::string joint_name,
            std::shared_ptr<double> force,
            double noise_thresh = 0.0,
            double target_force = 1.0,
            double init_k = 1,
            double min_vel = 0.01,
            double K_p = 1,
            double K_i = 0.001,
            double max_error_int = 1.1,
            unsigned int f_error_window = 200
            );

    void calculate(double p, double last_p_des, double dt);
    void on_transition();
    void reset_parameters(double time);
    void finish_iteration();
    void update_joint_states(double loop_time);

    double get_p_des(){return p_des_;};
    double get_v_des(){return v_des_;};

    // trajectory time from of joint in seconds
    double joint_time_;

    // controller finish mode: if true, we maintain goal force instead of finishing trajectory
    bool goal_maintain_ = false;

    // name of actuated joint
    std::string joint_name_;

    // pointer to force
    std::shared_ptr<double> force_;

    // configurable parameters
    double noise_thresh_;
    double target_force_;

    double init_k_;

    double min_vel_;

    double K_p_;
    double K_i_;

    double max_error_int_;
    unsigned int f_error_window_;

    // internal parameters
    int vel_limit_ = 0;
    int force_n_ = 0;

    std::deque<double> f_error_queue_;
    double error_integral_;

    double f_error_integral_;

    double last_force_;

    double k_;
    double p_;

    double p_des_;
    double v_des_;

    double force_T_;
    double p_T_;

    double delta_F_;
    double delta_p_;
    double delta_p_T_;

    double delta_p_vel_;
    double delta_p_force_;

    double last_p_des_;

    SENSOR_STATE sensor_state_;
    SENSOR_STATE last_sensor_state_;

}; // JointForceController

} // fcc

#endif //FORCE_CONTROLLER_CORE_FORCE_CONTROLLER_H
