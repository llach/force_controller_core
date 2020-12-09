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
            std::shared_ptr<float> force,
            float noise_thresh = 0.0,
            float target_force = 1.0,
            float init_k = 1,
            float min_vel = 0.01,
            float K_p = 1,
            float K_i = 0.001,
            float max_error_int = 1.1,
            int f_error_window = 200
            );

    void on_transition();
    void reset_parameters();
    void update_joint_states(double loop_time);

    // trajectory time from of joint in seconds
    double joint_time_;

    // controller finish mode: if true, we maintain goal force instead of finishing trajectory
    bool goal_maintain_ = false;

    // name of actuated joint
    std::string joint_name_;

    // pointer to force
    std::shared_ptr<float> force_;

    // configurable parameters
    float noise_thresh_;
    float target_force_;

    float init_k_;

    float min_vel_;

    float K_p_;
    float K_i_;

    float max_error_int_;
    int f_error_window_;

    // internal parameters
    int vel_limit_ = 0;
    int force_n_ = 0;

    std::deque<float> f_error_queue_;
    float error_integral_;

    float f_error_integral_;

    float last_force_;

    float k_;
    float p_;

    float force_T_;
    float p_T_;

    float delta_F_;
    float delta_p_;
    float delta_p_T_;

    float delta_p_vel_;
    float delta_p_force_;

    float des_vel_;
    float last_des_p_;

    SENSOR_STATE sensor_state_;
    SENSOR_STATE last_sensor_state_;

}; // JointForceController

} // fcc

#endif //FORCE_CONTROLLER_CORE_FORCE_CONTROLLER_H
