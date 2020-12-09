#include "force_controller_core/force_controller.h"

namespace fcc {

JointForceController::JointForceController(
        std::string joint_name,
        std::shared_ptr<double> force,
        double noise_thresh,
        double target_force,
        double init_k,
        double min_vel,
        double K_p,
        double K_i,
        double max_error_int,
        unsigned int f_error_window):
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

  p_des_ = 0.0;
  v_des_ = 0.0;

  error_integral_ = 0.0;

  f_error_queue_.erase(f_error_queue_.begin(), f_error_queue_.end());
  f_error_integral_ = 0.0;

  delta_F_ = 0.0;
  delta_p_ = 0.0;
  delta_p_T_ = 0.0;

  delta_p_vel_ = 0.0;
  delta_p_force_ = 0.0;
  
  last_p_des_ = 0.0;

  last_force_ = 0.0;

  joint_time_ = 0.0;
}

void JointForceController::update_joint_states(double dt){
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
      joint_time_ += dt;
    }
  }
}

void JointForceController::on_transition() {
  force_T_ = *force_;
  p_T_ = p_;
}

void JointForceController::calculate(double p, double last_p_des, double dt){
  double state_err = p - last_p_des;

  // prevent integral from exploding
  if (error_integral_ < max_error_int_){
    error_integral_ += state_err;
  }

  delta_p_T_ = (p_T_ - p);

  // estimate k
//  double k_bar_t = forces_ / delta_p_T_;
//  k_bar_t = std::max(k_bar_t, 10000.0);
//
//  if (!std::isinf(k_bar_t)){
//      k_ = lambda_*k_bar_t + (1-lambda_)*k_;
//  }

  // calculate new desired position
  double f_des = target_force_ - std::abs(*force_);
  double delta_p_force = (f_des / k_);
  delta_F_ = f_des;

  if (f_error_queue_.size() > f_error_window_)
    f_error_queue_.pop_back();

  f_error_queue_.push_front(delta_p_force);
  f_error_integral_ = std::accumulate(f_error_queue_.begin(), f_error_queue_.end(), 0.0);

  // --> use PI[D] controller here
  double delta_p_max = K_p_ * (min_vel_ * dt);
  delta_p_max += K_i_ * error_integral_;

  // enforce velocity limits
  vel_limit_ = 0;
  if (std::abs(delta_p_force) > std::abs(delta_p_max)){
    vel_limit_ = 1;
    delta_p_ = delta_p_max;
  } else {
    delta_p_ = delta_p_force;
  }

  // store debug info
  delta_p_vel_ = std::abs(delta_p_max);
  delta_p_force_ = std::abs(delta_p_force);

  // calculate new position and velocity
  p_des_ = p - delta_p_;
  v_des_ = (p_des_ - last_p_des_) / dt;
}

void JointForceController::finish_iteration(){
    last_force_ = *force_;
    last_sensor_state_ = sensor_state_;
    last_p_des_ = p_des_;
}

} // fcc