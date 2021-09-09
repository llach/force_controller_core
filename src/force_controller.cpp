#include "force_controller_core/force_controller.h"

namespace fcc {

JointForceController::JointForceController(
        std::string joint_name,
        std::shared_ptr<double> force,
        double noise_thresh,
        double target_force,
        double max_error,
        double init_k,
        double K_p,
        double K_i,
        bool closing_decrease):
    joint_name_(joint_name),
    force_(force),
    noise_thresh_(noise_thresh),
    target_force_(target_force),
    max_error_(max_error),
    init_k_(init_k),
    K_p_(K_p),
    K_i_(K_i) {
    closing_factor_ = closing_decrease ? -1 : 1;
    }

void JointForceController::reset_parameters(double time){
  sensor_state_ = SENSOR_STATE::NO_CONTACT;
  last_sensor_state_ = SENSOR_STATE::NO_CONTACT;

  k_ = init_k_;
  force_T_ = 0.0;
  q_T_ = 0.0;

  q_des_ = 0.0;
  v_des_ = 0.0;

  error_integral_ = 0.0;

  delta_F_ = 0.0;
  delta_q_ = 0.0;
  delta_q_T_ = 0.0;
  
  last_q_des_ = 0.0;

  last_force_ = 0.0;

  joint_time_ = time;
}

void JointForceController::update_joint_states(double dt, bool force_update_time){
  if (sensor_state_ < GOAL) { // if a joint has reached it's goal, we don't change sensor_state_s anymore
    if (std::abs(last_force_) <= noise_thresh_ && std::abs(*force_) > noise_thresh_) {
      sensor_state_ = GOT_CONTACT;
    } else if (std::abs(last_force_) > noise_thresh_ && std::abs(*force_) > noise_thresh_) {
      sensor_state_ = IN_CONTACT;
    }

    // no contact -> follow trajectory
    if (sensor_state_ == NO_CONTACT || force_update_time) {
      // proceeding like this could cause jerking joints. better: find joint_t which is closest to current joint_val and continue from there
      joint_time_ += dt;
    }
  }
}

void JointForceController::on_transition() {
  force_T_ = *force_;
  q_T_ = q_;
}

void JointForceController::calculate(double q, double dt){
  delta_q_T_ = (q_T_ - q);

  // calculate new desired position
  delta_F_ = target_force_ - *force_;
  double delta_q_force = (delta_F_ / k_);

  error_integral_ += delta_q_force * dt;
  if (error_integral_ > max_error_)
    error_integral_ = max_error_;
  delta_q_ = K_p_ * delta_q_force + K_i_ * error_integral_;

  // calculate new position and velocity
  q_des_ = q + closing_factor_ * delta_q_;
  v_des_ = (q_des_ - last_q_des_) / dt;
}

void JointForceController::finish_iteration(){
    last_force_ = *force_;
    last_sensor_state_ = sensor_state_;
    last_q_des_ = q_des_;
}

} // fcc