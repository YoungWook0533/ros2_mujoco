#ifndef FR3_CONTROLLER_HPP
#define FR3_CONTROLLER_HPP

#include <Eigen/Dense>
#include "robot_data.h"

using namespace Eigen;

namespace FR3Controller
{
    class Controller
    {
    public:
        Controller(double dt, RobotData* rd);
        ~Controller();

        // VectorXd tmpControl();
        VectorXd PDJointControl(const VectorXd& desired_q, const VectorXd& desired_qdot);
        VectorXd PDTaskControl(const VectorXd& desired_x, const VectorXd& desired_xdot);
        VectorXd QPIK(const VectorXd& desired_x, const VectorXd& desired_xdot);

        double getBlendingCoeff(double val, double limit1, double limit2);
        void saturatePosition(const VectorXd &q);
        void saturateVelocity(const VectorXd &dq);
        void saturateTorque(VectorXd &tau);
        VectorXd reviseTorque(const VectorXd &tau, const VectorXd &q, const VectorXd &dq);
        
        bool positionViolation(const Ref<const VectorXd> &q);
        bool velocityViolation(const Ref<const VectorXd> &dq);
        bool cartesianViolation(const Isometry3d &T);
        VectorXd generateSafetyTorque(const bool safety_enabled, const Isometry3d &T, const Ref<const VectorXd> &q, const Ref<const VectorXd> &dq, const Ref<const VectorXd> &tau);

        VectorXd KeyboardCtrl(const bool is_mode_changed, const VectorXd& desired_vel);

    private :
        double dt_;
        RobotData* robot_data_;

        MatrixXd M_T_;
        MatrixXd K_T_;
        MatrixXd B_T_;
        VectorXd Kp_;
        VectorXd Kd_;

        enum LimitState { SAFE = 0, MIN_SOFT = 1, MAX_SOFT = 2, MIN_HARD = 3, MAX_HARD = 4 };
        enum VelLimitState { V_SAFE = 0, MIN_SOFT_VEL = 1, MAX_SOFT_VEL = 2, MIN_HARD_VEL = 3, MAX_HARD_VEL = 4 };

        // Joint constraints
        VectorXd q_min_;
        VectorXd q_max_;
        VectorXd qdot_limit_;
        VectorXd tau_limit_;

        Matrix4d x_init_; 
        VectorXd q_init_;

        VectorXd joint_position_max;
        VectorXd joint_position_min;
        VectorXd joint_velocity_limits;
        VectorXd joint_torques_limits;

        VectorXd last_cmd_vel_;

        // damping gains
        VectorXd kv_safety;

        VectorXd pos_zones;
        VectorXd vel_zones;

        VectorXd pos_limit_flag;
        VectorXd vel_limit_flag;

        const Vector3d monitoring_point_ee_frame = Vector3d(0.0, 0.0, -0.15);
        const double safety_plane_z_coordinate = 0.28;
        const double safety_cylinder_radius = 0.28;
        const double safety_cylinder_height = 0.6;
        bool safety_mode_flag = false;
        VectorXd limited_joints;
        VectorXd x_dot_lfp_; 

        // zone 1 and 2 definitions subject to tuning
        double angle_tol = 1 * M_PI / 180;  // rad 
        double vel_tol = 0.1;               // rad/s (0.1 = 5 deg/s)
        double q_tol = 1e-1 * M_PI / 180;  

        VectorXd soft_min_angles;
        VectorXd soft_max_angles;
        VectorXd hard_min_angles;
        VectorXd hard_max_angles;
        VectorXd soft_min_joint_velocity_limits;
        VectorXd hard_min_joint_velocity_limits;
        VectorXd soft_max_joint_velocity_limits;
        VectorXd hard_max_joint_velocity_limits;

        double theta_z_;
        MatrixXd x_d_;

    };
} // namespace Controller

#endif // FR3_CONTROLLER_HPP