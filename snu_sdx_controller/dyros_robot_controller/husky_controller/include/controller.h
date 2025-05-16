#ifndef Husky_CONTROLLER_HPP
#define Husky_CONTROLLER_HPP

#include <Eigen/Dense>
#include "robot_data.h"

using namespace Eigen;

namespace HuskyController
{
    class Controller
    {
    public:
        Controller(double dt, RobotData* rd, const std::string& yaml_path);
        ~Controller();

        void loadConfigFromYaml(const std::string& yaml_file);

        // VectorXd tmpControl();
        VectorXd IK(const Vector3d& desired_base_velocity);
        Vector3d FK(const VectorXd& desired_wheel_vel);
        VectorXd VelocityCommand(const Vector3d& desired_base_vel, const Vector3d& current_base_vel);

    private :
        double dt_;
        RobotData* robot_data_;

        std::vector<std::string> wheel_names_;

        double base_width_;
        double wheel_radius_;
        double lin_vel_limit_;
        double ang_vel_limit_;
        double lin_acc_limit_;
        double ang_acc_limit_;

    };
} // namespace Controller

#endif // Husky_CONTROLLER_HPP