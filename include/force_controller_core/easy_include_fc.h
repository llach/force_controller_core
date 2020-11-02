// FTC enum CONTROLLER_STATE
using force_controller::ForceTrajectoryController<TactileSensors>::TRAJECTORY_EXEC;
using force_controller::ForceTrajectoryController<TactileSensors>::TRANSITION;
using force_controller::ForceTrajectoryController<TactileSensors>::FORCE_CTRL;

// FTC enum SENSOR_STATE
using force_controller::ForceTrajectoryController<TactileSensors>::NO_CONTACT;
using force_controller::ForceTrajectoryController<TactileSensors>::LOST_CONTACT;
using force_controller::ForceTrajectoryController<TactileSensors>::GOT_CONTACT;
using force_controller::ForceTrajectoryController<TactileSensors>::IN_CONTACT;
using force_controller::ForceTrajectoryController<TactileSensors>::GOAL;
using force_controller::ForceTrajectoryController<TactileSensors>::VIOLATED;

using force_controller::ForceTrajectoryController<TactileSensors>::sensors_;

using force_controller::ForceTrajectoryController<TactileSensors>::current_state_;
using force_controller::ForceTrajectoryController<TactileSensors>::desired_state_;

using force_controller::ForceTrajectoryController<TactileSensors>::forces_;
using force_controller::ForceTrajectoryController<TactileSensors>::max_forces_;
using force_controller::ForceTrajectoryController<TactileSensors>::last_forces_;
using force_controller::ForceTrajectoryController<TactileSensors>::thresh_forces_;

using force_controller::ForceTrajectoryController<TactileSensors>::name_;
using force_controller::ForceTrajectoryController<TactileSensors>::num_sensors_;

using force_controller::ForceTrajectoryController<TactileSensors>::NOISE_THRESH;

using force_controller::ForceTrajectoryController<TactileSensors>::k_;
using force_controller::ForceTrajectoryController<TactileSensors>::forces_T_;
using force_controller::ForceTrajectoryController<TactileSensors>::pos_T_;

using force_controller::ForceTrajectoryController<TactileSensors>::delta_F_;
using force_controller::ForceTrajectoryController<TactileSensors>::delta_p_;

using force_controller::ForceTrajectoryController<TactileSensors>::sensor_states_;
using force_controller::ForceTrajectoryController<TactileSensors>::last_sensor_states_;
using force_controller::ForceTrajectoryController<TactileSensors>::m_statestring_;

using force_controller::ForceTrajectoryController<TactileSensors>::c_state_;

using force_controller::ForceTrajectoryController<TactileSensors>::joint_names_;
using force_controller::ForceTrajectoryController<TactileSensors>::joints_;

using force_controller::ForceTrajectoryController<TactileSensors>::curr_time_;