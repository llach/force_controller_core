#include "force_controller_core/force_controller.h"

namespace fcc {

JointForceController::JointForceController(
        std::string joint_name,
        std::shared_ptr<float> force,
        float noise_thresh,
        float target_force,
        float init_k,
        float min_vel,
        float K_p,
        float K_i,
        float max_error_int,
        int f_error_window):
    joint_name_(joint_name),
    force_(force),
    noise_thresh_(noise_thresh),
    target_force_(target_force),
    init_k_(init_k),
    min_vel_(min_vel),
    K_p_(K_p),
    K_i_(K_i),
    max_error_int_(max_error_int),
    f_error_window_(f_error_window)
    {}

void JointForceController::reset_parameters(){
  sensor_state_ = SENSOR_STATE::NO_CONTACT;
  last_sensor_state_ = SENSOR_STATE::NO_CONTACT;

  k_ = init_k_;
  force_T_ = 0.0;
  p_T_ = 0.0;

  error_integral_ = 0.0;

  f_error_queue_.erase(f_error_queue_.begin(), f_error_queue_.end());
  f_error_integral_ = 0.0;

  delta_F_ = 0.0;
  delta_p_ = 0.0;
  delta_p_T_ = 0.0;

  delta_p_vel_ = 0.0;
  delta_p_force_ = 0.0;

  des_vel_ = 0.0;
  last_des_p_ = 0.0;

  last_force_ = 0.0;

  joint_time_ = 0.0;
}

void JointForceController::update_joint_states(double loop_time){
  if (sensor_state_ < GOAL) { // if a joint has reached it's goal, we don't change sensor_state_s anymore
    if (std::abs(last_force_) <= noise_thresh_ && std::abs(*force_) > noise_thresh_) {
      sensor_state_ = GOT_CONTACT;
    } else if (std::abs(last_force_) > noise_thresh_ && std::abs(*force_) <= noise_thresh_) {
      sensor_state_ = LOST_CONTACT;
    } else if (std::abs(last_force_) > noise_thresh_ && std::abs(*force_) > noise_thresh_) {
      sensor_state_ = IN_CONTACT;
    } else {
      sensor_state_ = NO_CONTACT;
    }

    // no contact -> follow trajectory
    if (sensor_state_ <= LOST_CONTACT) {
      // proceeding like this could cause jerking joints. better: find joint_t which is closest to current joint_val and continue from there
      joint_time_ += loop_time;
    }
  }
}

void JointForceController::on_transition() {
  force_T_ = *force_;
  p_T_ = p_;
}

} // fcc