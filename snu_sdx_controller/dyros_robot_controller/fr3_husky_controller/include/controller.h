#ifndef FR3Husky_CONTROLLER_HPP
#define FR3Husky_CONTROLLER_HPP

#include <Eigen/Dense>
#include "robot_data.h"

using namespace Eigen;

namespace FR3HuskyController
{
    class Controller
    {
    public:
        Controller(double dt, RobotData* rd, const std::string& yaml_path);
        ~Controller();

        void loadConfigFromYaml(const std::string& yaml_file);

        VectorXd PDJointControl(const VectorXd& desired_q, const VectorXd& desired_qdot);

        // VectorXd tmpControl();
        VectorXd IK(const Vector3d& desired_base_velocity);
        Vector3d FK(const VectorXd& desired_wheel_vel);
        VectorXd VelocityCommand(const Vector3d& desired_base_vel, const Vector3d& current_base_vel);

    private :
        double dt_;
        RobotData* robot_data_;

        std::vector<std::string> wheel_names_;

        MatrixXd M_T_;
        MatrixXd K_T_;
        MatrixXd B_T_;
        VectorXd Kp_;
        VectorXd Kd_;

        double base_width_;
        double wheel_radius_;
        double lin_vel_limit_;
        double ang_vel_limit_;
        double lin_acc_limit_;
        double ang_acc_limit_;

    };
} // namespace Controller

#endif // FR3Husky_CONTROLLER_HPP