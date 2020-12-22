#include <cmath>
#include <string>
#include <iostream>

#include <yarp/os/all.h>
#include <yarp/math/Math.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IControlLimits2.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPositionControl.h>

#include "force_controller_core/force_controller.h"

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

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

void JointForceController::reset_parameters(double time){
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

    joint_time_ = time;
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

class jointControl: public RFModule {

private:

    /////////////////////////////////////////

    BufferedPort<Bottle>  head_outPort;
    PolyDriver            head_jointDriver;
    IControlLimits       *head_ilim;
    IEncoders            *head_ienc;
    IControlMode         *head_imod;
    IPositionControl     *head_iposd;

    BufferedPort<Bottle> torso_outPort;
    PolyDriver           torso_jointDriver;
    IControlLimits       *torso_ilim;
    IEncoders            *torso_ienc;
    IControlMode         *torso_imod;
    IPositionControl     *torso_iposd;

    int head_joint;
    double head_min_lim, head_max_lim;
    double head_pos, head_init_pos;
    double head_p0, head_p1;
    double head_speed;

    /////////////////////////////////////////

    int torso_joint;
    double torso_min_lim, torso_max_lim;
    double torso_pos, torso_init_pos;
    double torso_p0, torso_p1;
    double torso_speed;

    /////////////////////////////////////////
    // pointer to force vector. Written to by TactileSensors and read by ForceController
    std::vector<std::shared_ptr<double>> forces_;

    // list of joint-level force controllers
    std::vector<fcc::JointForceController> jfc_;

    double period;

public:
    bool configure(ResourceFinder &rf){
        // get simulation true or false
        int sim   = rf.check("simulation", Value(1)).asInt();

        std::shared_ptr<double> fp = std::make_shared<double>(0.0);
        fcc::JointForceController jfc("joint_whatever",
                                      fp,
                                      0.25, // force threshold
                                      2.2, // target force
                                      875, // initial k
                                      0.01, // minimum velocity
                                      5, // K_p
                                      0.001, // K_i
                                      1.1, // maximum error integral value
                                      200 // force error integral windows length
                                      );
        jfc_.push_back(jfc);
        forces_.push_back(fp);

        // get parameters
        setName((rf.check("name", Value("/jointSinMotion")).asString()).c_str());


        /////////////////////////////////////////
        std::string head_body_part = "head";
        head_joint                 = 2;
        head_p0                    = rf.check("head_p0",Value(-42.0)).asDouble();
        head_p1                    = rf.check("head_p1",Value(42.0)).asDouble();
        head_speed                 = rf.check("head_speed",Value(15.0)).asDouble();

        /////////////////////////////////////////
        std::string torso_body_part = "torso";
        torso_joint                 = 0;
        torso_p0                    = rf.check("torso_p0",Value(48.0)).asDouble();
        torso_p1                    = rf.check("torso_p1",Value(-48.0)).asDouble();
        torso_speed                 = rf.check("torso_speed",Value(15.0)).asDouble();

        /////////////////////////////////////////
        period = rf.check("period", Value(1.0)).asDouble();

        // open communication with encoders
        Property optHead;
        optHead.put("device","remote_controlboard");
        optHead.put("local","/encReader/" + getName() + "/" + head_body_part);

        if (sim) {
            optHead.put("remote","/icubSim/" + head_body_part);
            if (!head_jointDriver.open(optHead)) {
                yError() << "Unable to connect to /icubSim/" << head_body_part;
                return false;
            }
        } else {
            optHead.put("remote","/icub/" + head_body_part);
            if (!head_jointDriver.open(optHead)) {
                yError() << "Unable to connect to /icub/" << head_body_part;
                return false;
            }
        }

        // open communication with encoders
        Property optTorso;
        optTorso.put("device","remote_controlboard");
        optTorso.put("local","/encReader/" + getName() + "/" + torso_body_part);

        if (sim) {
            optTorso.put("remote","/icubSim/" + torso_body_part);
            if (!torso_jointDriver.open(optTorso)) {
                yError() << "Unable to connect to /icubSim/" << torso_body_part;
                return false;
            }
        } else {
            optTorso.put("remote","/icub/" + torso_body_part);
            if (!torso_jointDriver.open(optTorso)) {
                yError() << "Unable to connect to /icub/" << torso_body_part;
                return false;
            }
        }
        /////////////////////////////////////////////////

        bool ok = true;
        ok = ok && head_jointDriver.view(head_ienc);
        ok = ok && head_jointDriver.view(head_ilim);
        ok = ok && head_jointDriver.view(head_imod);
        ok = ok && head_jointDriver.view(head_iposd);

        if (!ok){
            yError() << "Unable to open views";
            return false;
        }

        ok = true;
        ok = ok && torso_jointDriver.view(torso_ienc);
        ok = ok && torso_jointDriver.view(torso_ilim);
        ok = ok && torso_jointDriver.view(torso_imod);
        ok = ok && torso_jointDriver.view(torso_iposd);

        if (!ok){
            yError() << "Unable to open views";
            return false;
        }

        // open output port
        if (!head_outPort.open("/jointSinMotion/" + getName() + "/pos_head:o")) {
            yError() << "Unable to open jointSinMotion port";
            return false;
        }

        // open output port
        if (!torso_outPort.open("/jointSinMotion/" + getName() + "/pos_torso:o")) {
            yError() << "Unable to open jointSinMotion port";
            return false;
        }

        // compute joint limits
        head_ilim->getLimits(head_joint,&head_min_lim,&head_max_lim);
        yWarning() << "joint min limit: " << head_min_lim << " -   joint max limit: " << head_max_lim;

        // compute joint limits
        torso_ilim->getLimits(torso_joint,&torso_min_lim,&torso_max_lim);
        yWarning() << "joint min limit: " << torso_min_lim << " -   joint max limit: " << torso_max_lim;

        // set control mode
        head_imod->setControlMode(head_joint,VOCAB_CM_POSITION);
        // set control mode
        torso_imod->setControlMode(torso_joint,VOCAB_CM_POSITION);

        // move head joint to center
        head_init_pos = (head_max_lim + head_min_lim) / 2;
        head_iposd->setRefSpeed(head_joint,10);
        head_iposd->positionMove(head_joint, head_init_pos);
        yInfo() << "Waiting 3 seconds to reach head_init_pose";
        Time::delay(3);

        // move torso joint to center
        torso_init_pos = (torso_max_lim + torso_min_lim) / 2;
        torso_iposd->setRefSpeed(torso_joint,10);
        torso_iposd->positionMove(torso_joint, torso_init_pos);
        yInfo() << "Waiting 3 seconds to reach torso_init_pose";
        Time::delay(3);

        // check that amplitude !> motion allowed by the robot
        head_ienc->getEncoder(head_joint,&head_pos);
        head_iposd->setRefSpeed(head_joint,head_speed);

        // check that amplitude !> motion allowed by the robot
        torso_ienc->getEncoder(torso_joint,&torso_pos);
        torso_iposd->setRefSpeed(torso_joint,torso_speed);

        Time::delay(1);

        return true;
    }

    bool updateModule(){
        // generate target and sed command
        double t = Time::now();

        head_iposd->positionMove(head_joint,head_p0);
        torso_iposd->positionMove(torso_joint,torso_p0);
        Time::delay(8);
        head_iposd->positionMove(head_joint,head_p1);
        torso_iposd->positionMove(torso_joint,torso_p1);
        Time::delay(8);

        return true;
    }

    double getPeriod(){
        return period;
    }

    bool interruptModule(){
        yInfo() << "Interrupting module ...";
        head_outPort.interrupt();

        return RFModule::interruptModule();
    }

    bool close(){
        yInfo() << "Closing the module...";
        head_jointDriver.close();
        head_outPort.close();

        yInfo() << "...done!";
        return RFModule::close();
    }

};


int main(int argc, char* argv[]) {
    Network yarp;
    if(!yarp.checkNetwork()) {
        yError() << "Network not found";
        return -1;
    }

    ResourceFinder rf;
    rf.configure(argc, argv);

    jointControl jCtrl;
    return jCtrl.runModule(rf);
}
