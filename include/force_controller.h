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

#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <trajectory_interface/quintic_spline_segment.h>

#include <joint_trajectory_controller/joint_trajectory_segment.h>

namespace force_controller {

    template <class TactileSensors>
    class ForceTrajectoryController
            : public joint_trajectory_controller::JointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
              hardware_interface::PositionJointInterface>
{
    void goalCB(GoalHandle gh) override;
    void update(const ros::Time& time, const ros::Duration& period) override;

protected:
    bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& root_nh,
              ros::NodeHandle& controller_nh) override;

    virtual void update_sensors() = 0;
    virtual bool check_controller_transition() = 0;
    virtual void publish_debug_info() = 0;
    virtual void force_finished() = 0;

    std::string name_ = "force_controller";

    int num_sensors_ = 0;

    typedef std::shared_ptr<TactileSensors> TactileSensorsPtr;
    TactileSensorsPtr sensors_;

    // sample times for joints
    std::vector<ros::Time> joint_times_;

    // force vectors
    float NOISE_THRESH = 0.05;

    std::shared_ptr<std::vector<float>> forces_;
    std::shared_ptr<std::vector<float>> max_forces_;
    std::shared_ptr<std::vector<float>> last_forces_;
    std::shared_ptr<std::vector<float>> thresh_forces_;

    double lambda_ = 0.9;
    std::shared_ptr<std::vector<float>> k_;
    std::shared_ptr<std::vector<float>> forces_T_;
    std::shared_ptr<std::vector<float>> pos_T_;

    std::shared_ptr<std::vector<float>> delta_F_;
    std::shared_ptr<std::vector<float>> delta_p_;

    // sensor state data
    enum SENSOR_STATE {NO_CONTACT, LOST_CONTACT, GOT_CONTACT, IN_CONTACT};
    std::vector<SENSOR_STATE> sensor_states_;

    std::vector<SENSOR_STATE> last_sensor_states_;

    std::map<SENSOR_STATE, std::string> m_statestring_ = {
            {NO_CONTACT, "no contact"},
            {GOT_CONTACT, "got contact"},
            {LOST_CONTACT, "lost contact"},
            {IN_CONTACT, "still in contact"}
    };

    // controller state data
    enum CONTROLLER_STATE {TRAJECTORY_EXEC, TRANSITION, FORCE_CTRL};
    CONTROLLER_STATE c_state_;
};
}

#include <force_controller_impl.h>

#endif //FORCE_CONTROLLER_CORE_FORCE_CONTROLLER_H
