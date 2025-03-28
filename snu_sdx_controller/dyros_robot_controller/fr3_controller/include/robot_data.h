#ifndef FR3_ROBOT_DATA_HPP
#define FR3_ROBOT_DATA_HPP

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

namespace FR3Controller
{
    class RobotData
    {
        public:
            RobotData(const std::string& urdf_path);
            ~RobotData();
            bool updateState(const VectorXd& q, const VectorXd& qdot, const VectorXd& tau);

            std::vector<std::string> getJointNames() { return joint_names_; }

            Matrix4d computePose(const VectorXd& q, const std::string& link_name=ee_name_); 
            MatrixXd computeJacobian(const VectorXd& q, const std::string& link_name=ee_name_);
            MatrixXd computeJacobianTimeVariation(const VectorXd& q, const VectorXd& qdot, const std::string& link_name=ee_name_);
            VectorXd computeVelocity(const VectorXd& q, const VectorXd& qdot, const std::string& link_name=ee_name_);
            MatrixXd computeMassMatrix(const VectorXd& q);
            VectorXd computeCoriolis(const VectorXd& q, const VectorXd& qdot);
            VectorXd computeGravity(const VectorXd& q);
            VectorXd computeNonlinearEffects(const VectorXd& q, const VectorXd& qdot);
            MatrixXd computeTaskMassMatrix(const VectorXd& q, const std::string& link_name=ee_name_);
            VectorXd computeTaskCoriolis(const VectorXd& q, const VectorXd& qdot, const std::string& link_name=ee_name_);
            VectorXd computeTaskGravity(const VectorXd& q, const std::string& link_name=ee_name_);
            VectorXd computeTaskNonlinearEffects(const VectorXd& q, const VectorXd& qdot, const std::string& link_name=ee_name_);

            Matrix4d getPose(const std::string& link_name=ee_name_); 
            MatrixXd getJacobian(const std::string& link_name=ee_name_);
            MatrixXd getJacobianTimeVariation(const std::string& link_name=ee_name_);
            VectorXd getVelocity(const std::string& link_name=ee_name_);
            MatrixXd getMassMatrix();
            VectorXd getCoriolis();
            VectorXd getGravity();
            VectorXd getNonlinearEffects();
            MatrixXd getTaskMassMatrix(const std::string& link_name=ee_name_);
            VectorXd getTaskCoriolis(const std::string& link_name=ee_name_);
            VectorXd getTaskGravity(const std::string& link_name=ee_name_);
            VectorXd getTaskNonlinearEffects(const std::string& link_name=ee_name_);

        private:
            bool updateKinematics(const VectorXd& q, const VectorXd& qdot);
            bool updateDynamics(const VectorXd& q, const VectorXd& qdot);

            // pinocchio data
            pinocchio::Model model_;
            pinocchio::Data data_;

            std::vector<std::string> joint_names_;

            // Joint space state
            VectorXd q_;    // joint angle
            VectorXd qdot_; // joint velocity
            VectorXd tau_;  // joint torque

            // Task space state
            static constexpr const char* ee_name_ = "fr3_link8"; // end-effector link name
            Matrix4d x_;     // pose of EE
            VectorXd xdot_;  // velocity of EE
            MatrixXd J_;     // jacobian of EE
            MatrixXd Jdot_;  // time derivative of jacobian of EE

            // Joint space Dynamics
            MatrixXd M_;     // inertia matrix
            MatrixXd M_inv_; // inverse of inertia matrix
            VectorXd g_;     // gravity forces
            VectorXd c_;     // centrifugal and coriolis forces
            VectorXd NLE_;   // nonlinear effects ( g_ + c_ )

            // Task space Dynamics
            MatrixXd M_ee_;     // inertia matrix
            MatrixXd M_ee_inv_; // inverse of inertia matrix
            VectorXd g_ee_;     // gravity forces
            VectorXd c_ee_;     // centrifugal and coriolis forces
            VectorXd NLE_ee_;   // nonlinear effects ( g_ + c_ )
    };
}

#endif // FR3_ROBOT_DATA_HPP