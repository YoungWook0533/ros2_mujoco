#ifndef FR3Husky_ROBOT_DATA_HPP
#define FR3Husky_ROBOT_DATA_HPP

#include <string>
#include <Eigen/Dense>

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>

using namespace Eigen;

namespace FR3HuskyController
{
    class RobotData
    {
        public:
            RobotData(const std::string& urdf_path);
            ~RobotData();

            void updateState(const VectorXd& wheel_pose, const VectorXd& wheel_vel, const VectorXd& wheel_torque);

            std::vector<std::string> getWheelNames() { return wheel_names_; }
            VectorXd getWheelPose();
            VectorXd getWheelVel();

        private:
            // bool updateKinematics(const VectorXd& q, const VectorXd& qdot);
            // bool updateDynamics(const VectorXd& q, const VectorXd& qdot);

            // pinocchio data
            pinocchio::Model model_;
            pinocchio::Data data_;

            std::vector<std::string> wheel_names_;

            VectorXd wheel_pose_;
            VectorXd wheel_vel_;
            VectorXd wheel_torque_;
    };
}

#endif // FR3Husky_ROBOT_DATA_HPP