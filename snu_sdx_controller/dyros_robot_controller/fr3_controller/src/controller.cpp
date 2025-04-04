#include "controller.h"
#include "robot_data.h"
#include "math_type_define.h"
#include <cmath>

namespace FR3Controller
{ 

    Controller::Controller(double dt, RobotData* rd)
    {
        dt_ = dt;
        robot_data_ = rd;

        q_min_.resize(7);
        q_min_ <<  -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
        q_max_.resize(7);
        q_max_ << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.897;

        // joint velocity limits
        qdot_limit_.resize(7);
        qdot_limit_ << 2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100;

        // joint torque limits
        tau_limit_.resize(7);
        tau_limit_ << 87, 87, 87, 87, 12, 12, 12;

        // For interpolation
        x_init_ = Matrix4d::Identity();

        // For custom controllers

        joint_position_max.resize(7);
        joint_position_min.resize(7);
        joint_velocity_limits.resize(7);
        joint_torques_limits.resize(7);

        // damping gains
        // kv_safety = {20.0, 20.0, 20.0, 15.0, 10.0, 10.0, 5.0};
        kv_safety.resize(7);
        kv_safety << 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 1.0;

        pos_zones.resize(2);
        vel_zones.resize(2);

        // zone definitions
        pos_zones << 6., 9.;  // hard, soft
        // vel_zones = {5., 7.};  // hard, soft
        vel_zones << 8., 10.;  // hard, soft  (8, 6)

        double default_sf = 0.98;  // max violation safety factor 
        for (int i = 0; i < 7; ++i) {
            joint_position_max[i] = default_sf * q_max_[i];
            joint_position_min[i] = default_sf * q_min_[i];
            joint_velocity_limits[i] = default_sf * qdot_limit_[i];
            joint_torques_limits[i] = default_sf * tau_limit_[i];
        }

        soft_min_angles.resize(7);
        soft_max_angles.resize(7);
        hard_min_angles.resize(7);
        hard_max_angles.resize(7);
        soft_min_joint_velocity_limits.resize(7);
        hard_min_joint_velocity_limits.resize(7);
        soft_max_joint_velocity_limits.resize(7);
        hard_max_joint_velocity_limits.resize(7);

        soft_min_angles.setZero();
        soft_max_angles.setZero();
        hard_min_angles.setZero();
        hard_max_angles.setZero();
        soft_min_joint_velocity_limits.setZero();
        hard_min_joint_velocity_limits.setZero();
        soft_max_joint_velocity_limits.setZero();
        hard_max_joint_velocity_limits.setZero();

        for (int i = 0; i < 7; ++i) {
            soft_min_angles[i] = joint_position_min[i] + pos_zones[1] * angle_tol;
            hard_min_angles[i] = joint_position_min[i] + pos_zones[0] * angle_tol;
            soft_max_angles[i] = joint_position_max[i] - pos_zones[1] * angle_tol;
            hard_max_angles[i] = joint_position_max[i] - pos_zones[0] * angle_tol;
            soft_min_joint_velocity_limits[i] = - joint_velocity_limits[i] + vel_zones[1] * vel_tol;
            hard_min_joint_velocity_limits[i] = - joint_velocity_limits[i] + vel_zones[0] * vel_tol;
            soft_max_joint_velocity_limits[i] = joint_velocity_limits[i] - vel_zones[1] * vel_tol;
            hard_max_joint_velocity_limits[i] = joint_velocity_limits[i] - vel_zones[0] * vel_tol;
        }

        pos_limit_flag.resize(7); 
        pos_limit_flag.setZero();
        vel_limit_flag.resize(7); 
        vel_limit_flag.setZero();
        limited_joints.resize(7); 
        limited_joints.setZero();

        x_dot_lfp_ = Eigen::VectorXd::Zero(6);
        theta_z_ = 0.0;
        x_d_ = Eigen::MatrixXd::Zero(4,4);
    }

    Controller::~Controller()
    {
    }

    // VectorXd Controller::tmpControl()
    // {
    //     return VectorXd::Zero(7);
    // }

    VectorXd Controller::PDJointControl(const VectorXd& desired_q, const VectorXd& desired_qdot, const double kp, const double kd)
    {
        VectorXd qddot_target = kp * (desired_q - robot_data_->getq()) + kd * (desired_qdot - robot_data_->getqdot());
        VectorXd tau_desired = robot_data_->getMassMatrix() * qddot_target + robot_data_->getNonlinearEffects();
        return tau_desired;
    }

    VectorXd Controller::PDTaskControl(const VectorXd& desired_x, const VectorXd& desired_xdot, const double kp, const double kd)
    {
        // Compute pose error
        VectorXd x_error(6);
        Quaterniond desired_orientation = AngleAxisd(desired_x[5], Vector3d::UnitZ()) *
                                                 AngleAxisd(desired_x[4], Vector3d::UnitY()) *
                                                 AngleAxisd(desired_x[3], Vector3d::UnitX());
        Quaterniond current_orientation(robot_data_->getPose().block<3, 3>(0, 0));
        if (desired_orientation.coeffs().dot(current_orientation.coeffs()) < 0.0) {
            current_orientation.coeffs() << -current_orientation.coeffs();
        }
        Quaterniond orientation_error = current_orientation * desired_orientation.inverse();
        AngleAxisd angle_axis_error(orientation_error);
        x_error.head<3>() = robot_data_->getPose().block<3, 1>(0, 3) - desired_x.head<3>();
        x_error.tail<3>() = angle_axis_error.axis() * angle_axis_error.angle();

        // Compute velocity error
        VectorXd xdot_error(6);
        xdot_error = robot_data_->getJacobian() * robot_data_->getqdot() - desired_xdot;
            
        VectorXd tau_task = robot_data_->getJacobian().transpose() * (robot_data_->getTaskMassMatrix() * (-kp * x_error - kd * xdot_error) + robot_data_->getTaskNonlinearEffects());

        double lambda = 0.3;
        MatrixXd J_transpose_pinv = robot_data_->getJacobian() * (robot_data_->getJacobian().transpose() * robot_data_->getJacobian() + lambda * lambda * MatrixXd::Identity(7, 7)).inverse();

        // Null-space projection matrix
        MatrixXd N = MatrixXd::Identity(7, 7) - robot_data_->getJacobian().transpose() * J_transpose_pinv;

        VectorXd null_task = -robot_data_->getqdot() ;

        VectorXd tau_null = N * (null_task);

        VectorXd tau_desired = tau_task + tau_null + robot_data_->getNonlinearEffects();
        return tau_desired;
    }

    // ----- <User Custom Controller Func(C++)> -----

    double Controller::getBlendingCoeff(double val, double limit1, double limit2)
    {
        if (limit2 == limit1)
            return 1.0;
        double alpha = (val - limit1) / (limit2 - limit1);
        if (alpha < 0.0) alpha = 0.0;
        if (alpha > 1.0) alpha = 1.0;
        return alpha;
    }

    void Controller::saturatePosition(const VectorXd &q)
    {
        pos_limit_flag.setZero();
        limited_joints.setZero();

        for (int i = 0; i < q.size(); ++i) {
            double curr_q = q[i];
            if (curr_q > soft_min_angles[i] && curr_q < soft_max_angles[i]) {
                pos_limit_flag[i] = SAFE;
            } 
            else if (curr_q < hard_min_angles[i]) {
                pos_limit_flag[i] = MIN_HARD;
                limited_joints[i] = 1;
            } 
            else if (curr_q < soft_min_angles[i]) {
                pos_limit_flag[i] = MIN_SOFT;
                limited_joints[i] = 1;
            } 
            else if (curr_q > hard_max_angles[i]) {
                pos_limit_flag[i] = MAX_HARD;
                limited_joints[i] = 1;
            } 
            else if (curr_q > soft_max_angles[i]) {
                pos_limit_flag[i] = MAX_SOFT;
                limited_joints[i] = 1;
            }
        }
    }

    void Controller::saturateVelocity(const VectorXd &dq)
    {
        for (int i = 0; i < dq.size(); ++i) {
            if (pos_limit_flag[i] == SAFE) {
                if (dq[i] > soft_min_joint_velocity_limits[i] && dq[i] < soft_max_joint_velocity_limits[i]) {
                    vel_limit_flag[i] = V_SAFE;
                } else if (dq[i] > hard_max_joint_velocity_limits[i]) {
                    vel_limit_flag[i] = MAX_HARD_VEL;
                    limited_joints[i] = 1;
                } else if (dq[i] > soft_max_joint_velocity_limits[i]) {
                    vel_limit_flag[i] = MAX_SOFT_VEL;
                    limited_joints[i] = 1;
                } else if (dq[i] < hard_min_joint_velocity_limits[i]) {
                    vel_limit_flag[i] = MIN_HARD_VEL;
                    limited_joints[i] = 1;
                } else if (dq[i] < soft_min_joint_velocity_limits[i]) {
                    vel_limit_flag[i] = MIN_SOFT_VEL;
                    limited_joints[i] = 1;
                }
            } else {
                vel_limit_flag[i] = V_SAFE;
            }
        }
    }

    VectorXd Controller::reviseTorque(const VectorXd &tau,
                                                const VectorXd &q,
                                                const VectorXd &dq)
    {
        saturatePosition(q);
        saturateVelocity(dq);

        VectorXd tau_limited = tau;

        for (int i = 0; i < q.size(); ++i) {
            // Position-related modifications.
            if (pos_limit_flag[i] == MIN_SOFT) {
                double alpha = getBlendingCoeff(q[i], soft_min_angles[i], hard_min_angles[i]);
                tau_limited[i] = tau[i] - alpha * kv_safety[i] * dq[i];
            } else if (pos_limit_flag[i] == MAX_SOFT) {
                double alpha = getBlendingCoeff(q[i], soft_max_angles[i], hard_max_angles[i]);
                tau_limited[i] = tau[i] - alpha * kv_safety[i] * dq[i];
            } else if (pos_limit_flag[i] == MIN_HARD) {
                double alpha = getBlendingCoeff(q[i], hard_min_angles[i], q_min_[i]);
                double tau_hold = tau_limit_[i];
                tau_limited[i] = (1 - std::pow(alpha, 2)) * tau[i] + std::pow(alpha, 2) * tau_hold - kv_safety[i] * dq[i];
            } else if (pos_limit_flag[i] == MAX_HARD) {
                double alpha = getBlendingCoeff(q[i], hard_max_angles[i], q_max_[i]);
                double tau_hold = -tau_limit_[i];
                tau_limited[i] = (1 - std::pow(alpha, 2)) * tau[i] + std::pow(alpha, 2) * tau_hold - kv_safety[i] * dq[i];
            }
            // Velocity-related modifications.
            else if (vel_limit_flag[i] == MIN_SOFT_VEL) {
                double alpha = getBlendingCoeff(dq[i], soft_min_joint_velocity_limits[i], hard_min_joint_velocity_limits[i]);
                tau_limited[i] = (1 - std::pow(alpha, 1)) * tau[i];
            } else if (vel_limit_flag[i] == MAX_SOFT_VEL) {
                double alpha = getBlendingCoeff(dq[i], soft_max_joint_velocity_limits[i], hard_max_joint_velocity_limits[i]);
                tau_limited[i] = (1 - std::pow(alpha, 1)) * tau[i];
            } else if (vel_limit_flag[i] == MIN_HARD_VEL) {
                double alpha = getBlendingCoeff(dq[i], hard_min_joint_velocity_limits[i], -qdot_limit_[i]);
                tau_limited[i] = std::pow(alpha, 4) * tau_limit_[i] * 1e-2;
            } else if (vel_limit_flag[i] == MAX_HARD_VEL) {
                double alpha = getBlendingCoeff(dq[i], hard_max_joint_velocity_limits[i], qdot_limit_[i]);
                tau_limited[i] = -std::pow(alpha, 4) * tau_limit_[i] * 1e-2;
            }
        }

        return tau_limited;
    }

    bool Controller::positionViolation(const Ref<const VectorXd> &q)
    {
        bool is_violated = false;

        for(int i = 0; i < 7 ; i++){
            if(q[i] > joint_position_max[i])
            {
                is_violated = true;
                std::cout << "WARNING : Soft joint upper limit violated on joint " 
                << i << ", engaging safety mode" << std::endl;
            
            }
            if (q[i] < joint_position_min[i])
            {
                is_violated = true;
                std::cout << "WARNING : Soft joint lower limit violated on joint " 
                << i << ", engaging safety mode" << std::endl;
            }
        }   

        return is_violated;
        
    }

    bool Controller::velocityViolation(const Ref<const VectorXd> &dq)
    {
        bool is_violated = false;

        for(int i = 0; i < 7; i ++)
        {
            if (abs(dq[i]) > joint_velocity_limits[i])
            {
                is_violated = true;
                std::cout << "WARNING : Soft velocity limit violated on joint " 
                << i << ", engaging safety mode" << std::endl;
            }
        }

        return is_violated;
    }

    bool Controller::cartesianViolation(const Isometry3d &T)
    {
        bool is_violated = false;

        // cartesian checks
        Vector3d monitoring_point;
        monitoring_point = T.linear()*monitoring_point_ee_frame + T.translation();

        double radius_square = monitoring_point(0)*monitoring_point(0) + monitoring_point(1)*monitoring_point(1);
        double z_ee = monitoring_point(2);

        // lower plane
        if(z_ee < safety_plane_z_coordinate)
        {
            is_violated = true;
            std::cout << "WARNING : End effector too low, engaging safety mode" << std::endl;
            std::cout << "position of monitoring point : " << monitoring_point.transpose() << std::endl;     
        }
        // cylinder
        if(z_ee < safety_cylinder_height && radius_square < safety_cylinder_radius*safety_cylinder_radius)
        {
            is_violated = true;
            std::cout << "WARNING : End effector too close to center of workspace, engaging safety mode" << std::endl;
            std::cout << "position of monitoring point : " << monitoring_point.transpose() << std::endl;    
        }
        
        return is_violated;
    }

    VectorXd Controller::generateSafetyTorque(const bool safety_enabled,
                                                      const Isometry3d &T,
                                                      const Ref<const VectorXd> &q,
                                                      const Ref<const VectorXd> &dq,
                                                      const Ref<const VectorXd> &tau)
    {
    bool pos_safty, vel_safty;
    VectorXd safety_torque;

    safety_torque = tau;

    pos_safty = positionViolation(q);
    vel_safty = velocityViolation(dq);

    if(pos_safty || vel_safty) safety_mode_flag = true;

    if (safety_enabled)
        safety_mode_flag = cartesianViolation(T);

        if(safety_mode_flag)
        {
            x_d_ = robot_data_->getPose();
        }
        return safety_torque;

    }


    VectorXd Controller::KeyboardCtrl(const bool init, const VectorXd& cmd_vel)
    {
        // Initialization Block
        if (init) {
            x_init_ = robot_data_->getPose();  
            x_d_ = x_init_;
        }

        double alpha = 0.03045; // 10Hz cutoff frequency

        // Low-Pass Filter on Linear Velocity
        for (int i = 0; i < 3; i++){
            x_dot_lfp_(i) = alpha * cmd_vel(i) + (1.0 - alpha) * x_dot_lfp_(i);
        }
        for (int i = 3; i < 6; i++){
            x_dot_lfp_(i) = 0.0;
        }

        // Update Desired Pose
        x_d_.block<3,1>(0,3) += x_dot_lfp_.head(3) * dt_;

        theta_z_ += cmd_vel(5) * dt_;
        Matrix3d R_d = DyrosMath::rotateWithZ(theta_z_);
        x_d_.block<3,3>(0,0) = R_d * x_init_.block<3,3>(0,0);

        VectorXd desired_x(6);
        desired_x.head(3) = x_d_.block<3,1>(0,3);
        desired_x.tail(3) = DyrosMath::rot2Euler(x_d_.block<3,3>(0,0));

        // std::cout<<"desired_x : "<< desired_x.transpose() <<std::endl;
        // std::cout<<"xdot : "<< xdot_.transpose() <<std::endl;

        VectorXd xdot_d = VectorXd::Zero(6);

        VectorXd tau_desired = PDTaskControl(desired_x, xdot_d, 1000, 100);

        VectorXi limited_joints(7);
        limited_joints.setZero();
        int n_limited = 0;
        for (int i = 0; i < 7; ++i) {
            if (robot_data_->getq()(i) < q_min_(i) || robot_data_->getq()(i) > q_max_(i) - 0.1) {
                limited_joints(i) = 1;
                n_limited++;
            }
        }

        if (n_limited > 0) {
            MatrixXd Js = MatrixXd::Zero(n_limited, 7);
            int row = 0;
            for (int i = 0; i < 7; ++i) {
                if (limited_joints(i) == 1) {
                    Js(row, i) = 1.0;
                    row++;
                }
            }

            MatrixXd M_inv_ = robot_data_->getMassMatrix().inverse();
            MatrixXd Lambda_s = (Js * M_inv_ * Js.transpose()).inverse();
            MatrixXd Jbar_s = M_inv_ * Js.transpose() * Lambda_s;
            MatrixXd N_s = MatrixXd::Identity(7, 7) - Jbar_s * Js;

            VectorXd tau_revised = reviseTorque(tau_desired, robot_data_->getq(), robot_data_->getqdot());
            tau_desired = tau_revised + N_s.transpose() * tau_desired;
        }
        else {
            tau_desired = tau_desired;
        }

        Isometry3d T;
        T.matrix() = x_d_;
        tau_desired = generateSafetyTorque(true, T, robot_data_->getq(), robot_data_->getqdot(), tau_desired);

        return tau_desired;
    }
}