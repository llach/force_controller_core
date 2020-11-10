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

#ifndef FORCE_CONTROLLER_CORE_FORCE_CONTROLLER_IMPL_H
#define FORCE_CONTROLLER_CORE_FORCE_CONTROLLER_IMPL_H

#include <stdexcept>

namespace force_controller {
    template <class TactileSensors>
    inline bool ForceTrajectoryController<TactileSensors>::init(hardware_interface::PositionJointInterface* hw,
                                                               ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
        ROS_INFO_NAMED(name_, "FTC init");

        bool ret = JointTrajectoryController::init(hw, root_nh, controller_nh);
        num_sensors_ = joints_.size();

        forces_= std::make_shared<std::vector<float>>(num_sensors_, 0.0);
        last_forces_= std::make_shared<std::vector<float>>(num_sensors_, 0.0);

        k_ = std::make_shared<std::vector<float>>(num_sensors_, 0.0);
        forces_T_ = std::make_shared<std::vector<float>>(num_sensors_, 0.0);
        pos_T_ = std::make_shared<std::vector<float>>(num_sensors_, 0.0);

        delta_F_ = std::make_shared<std::vector<float>>(num_sensors_, 0.0);
        delta_p_ = std::make_shared<std::vector<float>>(num_sensors_, 0.0);

        sensor_states_ = std::vector<SENSOR_STATE>(num_sensors_, SENSOR_STATE::NO_CONTACT);
        last_sensor_states_ = std::vector<SENSOR_STATE>(num_sensors_, SENSOR_STATE::NO_CONTACT);

        c_state_ = TRAJECTORY_EXEC;

      ROS_INFO_NAMED(name_, "reloading action server ...");
      // ROS API: Action interface
      action_server_.reset(new ActionServer(controller_nh_, "follow_joint_trajectory",
                                            boost::bind(&ForceTrajectoryController::goalCB,   this, _1),
                                            boost::bind(&ForceTrajectoryController::cancelCB, this, _1),
                                            false));
      action_server_->start();
      for (int i=0; i<joint_names_.size(); i++){
        ROS_INFO_STREAM_NAMED(name_, "JN" << i << " " << joint_names_[i]);
      }
      joint_times_.resize(num_sensors_);

        sensors_ = std::make_shared<TactileSensors>(root_nh, forces_);

        return ret;
    }

    template <class TactileSensors>
    inline void ForceTrajectoryController<TactileSensors>::cancelCB(GoalHandle gh) {
      ROS_INFO_NAMED(name_, "Canceling action goal");
      JointTrajectoryController::cancelCB(gh);
    }

    template <class TactileSensors>
    inline void ForceTrajectoryController<TactileSensors>::goalCB(GoalHandle gh) {
        ROS_INFO_NAMED(name_, "Received new action goal");

        // Precondition: Running controller
        if (!this->isRunning()) {
            ROS_ERROR_NAMED(name_, "Can't accept new action goals. Controller is not running.");
            control_msgs::FollowJointTrajectoryResult result;
            result.error_code =
                    control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;  // TODO: Add better error status to msg?
            gh.setRejected(result);
            return;
        }

        // If partial joints goals are not allowed, goal should specify all controller joints
        if (!allow_partial_joints_goal_) {
            if (gh.getGoal()->trajectory.joint_names.size() != joint_names_.size()) {
                ROS_ERROR_NAMED(name_, "Joints on incoming goal don't match the controller joints.");
                control_msgs::FollowJointTrajectoryResult result;
                result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
                gh.setRejected(result);
                return;
            }
        }

        // Goal should specify valid controller joints (they can be ordered differently). Reject if this is not the case
        using joint_trajectory_controller::internal::mapping; // todo error_string in newer JTC::internal?
        std::vector<unsigned int> mapping_vector = mapping(gh.getGoal()->trajectory.joint_names, joint_names_);

        if (mapping_vector.empty()) {
            ROS_ERROR_NAMED(name_, "Joints on incoming goal don't match the controller joints.");
            control_msgs::FollowJointTrajectoryResult result;
            result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
            gh.setRejected(result);
            return;
        }

        // Try to update new trajectory
        RealtimeGoalHandlePtr rt_goal(new RealtimeGoalHandle(gh));
        std::string error_string = "";  // todo upstream passed this one to updateTrajctoryCommand
        const bool update_ok = updateTrajectoryCommand(
                joint_trajectory_controller::internal::share_member(gh.getGoal(), gh.getGoal()->trajectory), rt_goal);
        rt_goal->preallocated_feedback_->joint_names = joint_names_;

        if (update_ok) {
            // Accept new goal
            preemptActiveGoal();
            gh.setAccepted();
            rt_active_goal_ = rt_goal;

            // Setup goal status checking timer
            goal_handle_timer_ =
                    controller_nh_.createTimer(action_monitor_period_, &RealtimeGoalHandle::runNonRealtime, rt_goal);
            goal_handle_timer_.start();

            reset_parameters();

        } else {
            // Reject invalid goal
            control_msgs::FollowJointTrajectoryResult result;
            result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
            result.error_string = error_string;
            gh.setRejected(result);
        }
    }

    template <class TactileSensors>
    inline void ForceTrajectoryController<TactileSensors>::reset_parameters(){
    /*
     * Reset controller / joint states & joint times
     */
      ROS_INFO_NAMED(name_, "Resetting controller and sensor states");
      c_state_ = TRAJECTORY_EXEC;
      sensor_states_ = std::vector<SENSOR_STATE>(num_sensors_, SENSOR_STATE::NO_CONTACT);
      last_sensor_states_ = std::vector<SENSOR_STATE>(num_sensors_, SENSOR_STATE::NO_CONTACT);

      k_ = std::make_shared<std::vector<float>>(num_sensors_, 0.0);
      forces_T_ = std::make_shared<std::vector<float>>(num_sensors_, 0.0);
      pos_T_ = std::make_shared<std::vector<float>>(num_sensors_, 0.0);

      delta_F_ = std::make_shared<std::vector<float>>(num_sensors_, 0.0);
      delta_p_ = std::make_shared<std::vector<float>>(num_sensors_, 0.0);

      for (int j = 0; j < forces_->size(); j++){
        (*last_forces_)[j] = 0.0;
        (*forces_)[j] = 0.0;
      }

      for (auto &t : joint_times_)
        t = time_data_.readFromRT()->uptime;
    }

    template <class TactileSensors>
    inline void ForceTrajectoryController<TactileSensors>::update(const ros::Time& time, const ros::Duration& period) {

        realtime_busy_ = true;
        // Get currently followed trajectory
        TrajectoryPtr curr_traj_ptr;
        curr_trajectory_box_.get(curr_traj_ptr);
        Trajectory& curr_traj = *curr_traj_ptr;

        // Update time data
        TimeData time_data;
        time_data.time = time;  // Cache current time
        time_data.period = period;  // Cache current control period
        time_data.uptime = time_data_.readFromRT()->uptime + period;  // Update controller uptime
        time_data_.writeFromNonRT(time_data);  // TODO: Grrr, we need a lock-free data structure here!

        curr_time_ = time;

        // NOTE: It is very important to execute the two above code blocks in the specified sequence: first get current
        // trajectory, then update time data. Hopefully the following paragraph sheds a bit of light on the rationale.
        // The non-rt thread responsible for processing new commands enqueues trajectories that can start at the _next_
        // control cycle (eg. zero start time) or later (eg. when we explicitly request a start time in the future).
        // If we reverse the order of the two blocks above, and update the time data first; it's possible that by the time we
        // fetch the currently followed trajectory, it has been updated by the non-rt thread with something that starts in
        // the
        // next control cycle, leaving the current cycle without a valid trajectory.

        // call child template class to update sensors
        update_sensors();

        /*
         * Update forces and sensor states.
         */
        RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);
        if (current_active_goal) {
          ROS_DEBUG_STREAM_NAMED(name_ + ".forces", "Forces: [" << (*forces_)[0] << ", " << (*forces_)[1] << "]");

          for (int j = 0; j < forces_->size(); j++) {
                auto &force = (*forces_)[j];
                auto &last_force = (*last_forces_)[j];
                auto &state = sensor_states_[j];
                auto &last_state = last_sensor_states_[j]; // debug purposes only, not used in logic
                auto &joint_time = joint_times_[j];

                if (state < GOAL) { // if a joint has reached it's goal, we don't change states anymore
                  if (std::abs(last_force) <= NOISE_THRESH && std::abs(force) > NOISE_THRESH) {
                    state = GOT_CONTACT;
                  } else if (std::abs(last_force) > NOISE_THRESH && std::abs(force) <= NOISE_THRESH) {
                    state = LOST_CONTACT;
                  } else if (std::abs(last_force) > NOISE_THRESH && std::abs(force) > NOISE_THRESH) {
                    state = IN_CONTACT;
                  } else {
                    state = NO_CONTACT;
                  }

                  // no contact -> do trajectory sampling.
                  if (state <= LOST_CONTACT || true) {
                    // TODO proceeding like this could cause jerking joints. better: find joint_t which is closest to current joint_val and continue from there
                    joint_time += period;
                  }
                }

                ROS_DEBUG_NAMED(name_ + ".sensorState",
                           "Sensor %d has state %s with force %.4f", j, m_statestring_[state].c_str(), force);

                if (state != last_state) {
                    ROS_INFO_NAMED(name_ + ".sensorStateChange",
                                   "Sensor %d's state changed from %s to %s. f_t-1 %.4f, f_t %.4f", j,
                                   m_statestring_[last_state].c_str(), m_statestring_[state].c_str(), last_force, force);
                }
            }

            // update controller state
            if (c_state_ == TRANSITION) {
                // if the transition was detected last time, we enter force_control from here onwards
                c_state_ = FORCE_CTRL;
            } else if (c_state_ == TRAJECTORY_EXEC) {
                if (check_controller_transition() && false) { // handled in child
                    ROS_INFO_NAMED(name_, "C TRANSITION!");
                    c_state_ = TRANSITION;

                    // Store reference data.
                    for (int j = 0; j < forces_->size(); j++){
                        (*forces_T_)[j] = (*forces_)[j];
                        (*pos_T_)[j] = current_state_.position[j];
                    }
                }
                // if we were to revert back to trajectory mode, here is the place to do so
            }
        }

        // Update desired state and state error
        for (unsigned int i = 0; i < joints_.size(); ++i) {
            current_state_.position[i] = joints_[i].getPosition();
            current_state_.velocity[i] = joints_[i].getVelocity();
            // There's no acceleration data available in a joint handle

            typename TrajectoryPerJoint::const_iterator segment_it;
            if (c_state_ > TRANSITION && std::abs((*forces_)[i]) > NOISE_THRESH && false) {
                ROS_INFO_STREAM_NAMED(name_, "IN FC " << i);
                double delta_p = current_state_.position[i] - (*pos_T_)[i];
                double k_bar_t = (*forces_)[i] / delta_p;

                if (!std::isinf(k_bar_t)){
                    (*k_)[i] = (1-lambda_)*k_bar_t + lambda_*(*k_)[i];
                } else {
                    k_bar_t = 0.0;
                }

                // if joint is at max force, we set it to hold the current position
                double f_des = (*max_forces_)[i] - (*forces_)[i];
                double p_des_ = (f_des / (*k_)[i]) + current_state_.position[i];

                // store debug info
                (*delta_F_)[i] = f_des;
                (*delta_p_)[i] = delta_p;

//                if (newF < goal_joint_forces_[i]){
//                    p_des_ += (newF/ k_bar_[i]);
//                }

                desired_joint_state_.position[0] = p_des_;
                desired_joint_state_.velocity[0] = current_state_.velocity[i]; // TODO this could probably be done smarter

            } else {
                segment_it =
                        sample(curr_traj[i], joint_times_[i].toSec(), desired_joint_state_);
                if (curr_traj[i].end() == segment_it) {
                    // Non-realtime safe, but should never happen under normal operation
                    ROS_ERROR_NAMED(
                            name_,
                            "Unexpected error: No trajectory defined at current time. Please contact the package maintainer.");
                    cancelCB(current_active_goal->gh_);
                    return;
                }
            }

            desired_state_.position[i] = desired_joint_state_.position[0];
            desired_state_.velocity[i] = desired_joint_state_.velocity[0];
            desired_state_.acceleration[i] = desired_joint_state_.acceleration[0];


            state_joint_error_.position[0] =
                    angles::shortest_angular_distance(current_state_.position[i], desired_joint_state_.position[0]);
            state_joint_error_.velocity[0] = desired_joint_state_.velocity[0] - current_state_.velocity[i];
            state_joint_error_.acceleration[0] = 0.0;

            state_error_.position[i] =
                    angles::shortest_angular_distance(current_state_.position[i], desired_joint_state_.position[0]);
            state_error_.velocity[i] = desired_joint_state_.velocity[0] - current_state_.velocity[i];
            state_error_.acceleration[i] = 0.0;

            // Check tolerances
            if (c_state_ <= TRANSITION) { // => Trajectory Execution || TODO this might cause non terminating controller if case 2
                const RealtimeGoalHandlePtr rt_segment_goal = segment_it->getGoalHandle();
                if (rt_segment_goal && rt_segment_goal == rt_active_goal_) {
                    // Check tolerances
                    if (time_data.uptime.toSec() < segment_it->endTime()) {
                        // Currently executing a segment: check path tolerances
                        const joint_trajectory_controller::SegmentTolerancesPerJoint<Scalar> &joint_tolerances =
                                segment_it->getTolerances();
                        if (!checkStateTolerancePerJoint(state_joint_error_, joint_tolerances.state_tolerance)) {
                            if (verbose_) {
                                ROS_ERROR_STREAM_NAMED(name_, "Path tolerances failed for joint: " << joint_names_[i]);
                                checkStateTolerancePerJoint(state_joint_error_, joint_tolerances.state_tolerance, true);

                            }
                            sensor_states_[i] = VIOLATED;
                            if (rt_segment_goal && rt_segment_goal->preallocated_result_) {
                                ROS_INFO_NAMED(name_, "Trajectory execution aborted (path tolerances)");
                                rt_segment_goal->preallocated_result_->error_code =
                                        control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
                                rt_segment_goal->setAborted(rt_segment_goal->preallocated_result_);
                                rt_active_goal_.reset();
                                successful_joint_traj_.reset();
                                reset_parameters();
                            } else {
                                ROS_ERROR_STREAM("rt_segment_goal->preallocated_result_ NULL Pointer");
                            }
                        }
                    } else if (segment_it == --curr_traj[i].end()) {
                      // this else if should activate once a joint trajectory reaches it's last segment. this should only happen if we have increased joint timings for joint[i]

                        if (verbose_)
                            ROS_INFO_STREAM_THROTTLE_NAMED(1, name_,
                                                            "Finished executing last segment, checking goal tolerances");

                        // Controller uptimegit st
                        const ros::Time uptime = time_data_.readFromRT()->uptime;

                        // Checks that we have ended inside the goal tolerances
                        const joint_trajectory_controller::SegmentTolerancesPerJoint<Scalar> &tolerances = segment_it->getTolerances();
                        const bool inside_goal_tolerances =
                                checkStateTolerancePerJoint(state_joint_error_, tolerances.goal_state_tolerance);

                        if (inside_goal_tolerances) {
                          successful_joint_traj_[i] = 1;
                          sensor_states_[i] = GOAL;

                          if (last_sensor_states_[i] != GOAL) {
                            ROS_INFO_NAMED(name_, "Joint %d inside goal tolerances!", i);
                          }
                        } else if (uptime.toSec() < segment_it->endTime() + tolerances.goal_time_tolerance) {
                            // Still have some time left to meet the goal state tolerances
                        } else {
                            if (verbose_) {
                                ROS_ERROR_STREAM_NAMED(name_, "Goal tolerances failed for joint: " << joint_names_[i]);
                                // Check the tolerances one more time to output the errors that occurs
                                checkStateTolerancePerJoint(state_joint_error_, tolerances.goal_state_tolerance, true);
                            }
                            sensor_states_[i] = VIOLATED;
                            if (rt_segment_goal) {
                                ROS_INFO_NAMED(name_, "Trajectory execution aborted (goal tolerances)");
                                rt_segment_goal->preallocated_result_->error_code =
                                        control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
                                rt_segment_goal->setAborted(rt_segment_goal->preallocated_result_);
                            } else {
                                ROS_ERROR_STREAM("rt_segment_goal->preallocated_result_ NULL Pointer");
                            }
                            rt_active_goal_.reset();
                            successful_joint_traj_.reset();
                        }
                    }
                }
            }
        }



        // If there is an active goal and all segments finished successfully then set goal as succeeded
        // current_active_goal is reused from above the state update
        if (current_active_goal && current_active_goal->preallocated_result_ &&
            successful_joint_traj_.count() == joints_.size()) {
            ROS_INFO_NAMED(name_, "Trajectory execution succeeded");
            current_active_goal->preallocated_result_->error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
            current_active_goal->setSucceeded(current_active_goal->preallocated_result_);
            rt_active_goal_.reset();
            successful_joint_traj_.reset();
            reset_parameters();
        } else if (current_active_goal && current_active_goal->preallocated_result_) {
            if (check_finished()) {
                ROS_INFO_NAMED(name_, "Non-trajectory success");

                for (unsigned int i = 0; i < joints_.size(); ++i) {
                    ROS_INFO_NAMED(name_, "Setting joint %d to %f", i, current_state_.position[i]); // TODO or hold_state()?
                    desired_state_.position[i] = current_state_.position[i];
                    desired_state_.velocity[i] = current_state_.velocity[i];
                    desired_state_.acceleration[i] = current_state_.acceleration[i];

                    state_joint_error_.position[0] = 0.0;
                    state_joint_error_.velocity[0] = 0.0;
                    state_joint_error_.acceleration[0] = 0.0;

                    state_error_.position[i] = 0.0;
                    state_error_.velocity[i] = 0.0;
                    state_error_.acceleration[i] = 0.0;
                }

                current_active_goal->preallocated_result_->error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
                current_active_goal->setSucceeded(current_active_goal->preallocated_result_);
                rt_active_goal_.reset();
                successful_joint_traj_.reset();
                reset_parameters();

                // Hold current position
                setHoldPosition(time_data.uptime);
            }
        }

        // Hardware interface adapter: Generate and send commands
        hw_iface_adapter_.updateCommand(time_data.uptime, time_data.period, desired_state_, state_error_);

        // Set action feedback
        if (rt_active_goal_ && rt_active_goal_->preallocated_feedback_) {
            rt_active_goal_->preallocated_feedback_->header.stamp = time_data_.readFromRT()->time;
            rt_active_goal_->preallocated_feedback_->desired.positions = desired_state_.position;
            rt_active_goal_->preallocated_feedback_->desired.velocities = desired_state_.velocity;
            rt_active_goal_->preallocated_feedback_->desired.accelerations = desired_state_.acceleration;
            rt_active_goal_->preallocated_feedback_->actual.positions = current_state_.position;
            rt_active_goal_->preallocated_feedback_->actual.velocities = current_state_.velocity;
            rt_active_goal_->preallocated_feedback_->error.positions = state_error_.position;
            rt_active_goal_->preallocated_feedback_->error.velocities = state_error_.velocity;
            rt_active_goal_->setFeedback(rt_active_goal_->preallocated_feedback_);

        }

      publish_debug_info();

      /*
       * Store data for next loop & cleanup.
       */
        for (int j = 0; j < forces_->size(); j++){
            (*last_forces_)[j] = (*forces_)[j];
            last_sensor_states_[j] = sensor_states_[j];
        }

        // Publish state
        publishState(time_data.uptime);
        realtime_busy_ = false;
    }
}

#endif //FORCE_CONTROLLER_CORE_FORCE_CONTROLLER_IMPL_H
