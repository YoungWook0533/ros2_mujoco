#include "robot_data.h"
#include <yaml-cpp/yaml.h>
#include <optional>


namespace FR3HuskyController
{
    RobotData::RobotData(const std::string& base_urdf_path, const std::string& arm_urdf_path, const std::string& yaml_path)
    {
        std::ifstream base_file(base_urdf_path);
        if (!base_file.good()) std::cout << "URDF file does not exist! : " << base_urdf_path << std::endl;
        std::ifstream arm_file(arm_urdf_path);
        if (!arm_file.good()) std::cout << "URDF file does not exist! : " << arm_urdf_path << std::endl;
    
        pinocchio::urdf::buildModel(base_urdf_path, base_model_);
        base_data_ = pinocchio::Data(base_model_);
        pinocchio::urdf::buildModel(arm_urdf_path, arm_model_);
        arm_data_ = pinocchio::Data(arm_model_);
                
        joint_names_ = arm_model_.names;
        joint_names_.erase(joint_names_.begin());  // Remove the first element "universe_joint"

        wheel_names_ = base_model_.names;
        wheel_names_.erase(wheel_names_.begin());  // Remove the first element "universe_joint"

        // Initialize joint space state
        q_ = VectorXd::Zero(arm_model_.nq);
        qdot_ = VectorXd::Zero(arm_model_.nq);
        tau_ = VectorXd::Zero(arm_model_.nq);

        // Initialize task space state
        x_ = Matrix4d::Identity();
        xdot_ = VectorXd::Zero(6);
        J_ = MatrixXd::Zero(6, arm_model_.nv);
        Jdot_ = MatrixXd::Zero(6, arm_model_.nv);

        // Initialize joint space dynamics
        M_ = MatrixXd::Zero(arm_model_.nq, arm_model_.nv);
        M_inv_ = MatrixXd::Zero(arm_model_.nq, arm_model_.nv);
        g_ = VectorXd::Zero(arm_model_.nq);       
        c_ = VectorXd::Zero(arm_model_.nq);       
        NLE_ = VectorXd::Zero(arm_model_.nq);       

        // Initialize task space dynamics
        M_ee_ = MatrixXd::Zero(6, 6);
        M_ee_inv_ = MatrixXd::Zero(6, 6);
        g_ee_ = VectorXd::Zero(6);       
        c_ee_ = VectorXd::Zero(6);       
        NLE_ee_ = VectorXd::Zero(6); 
        
        // Gains
        M_T_ = MatrixXd::Zero(6, 6);
        K_T_ = MatrixXd::Zero(6, 6);
        B_T_ = MatrixXd::Zero(6, 6);
        Kp_.resize(arm_model_.nq);
        Kd_.resize(arm_model_.nq);

        wheel_pose_ = VectorXd::Zero(wheel_names_.size());
        wheel_vel_ = VectorXd::Zero(wheel_names_.size());
        wheel_torque_ = VectorXd::Zero(wheel_names_.size());

        loadGainsFromYaml(yaml_path);

    }

    RobotData::~RobotData()
    {

    }

    void RobotData::loadGainsFromYaml(const std::string& yaml_file) {

        YAML::Node config = YAML::LoadFile(yaml_file);

        // Load task space gains
        const auto& task_space = config["task_space_gains"];
        VectorXd M_T_diag = VectorXd::Map(task_space["M_T"].as<std::vector<double>>().data(), 6);
        VectorXd K_T_diag = VectorXd::Map(task_space["K_T"].as<std::vector<double>>().data(), 6);
        VectorXd B_T_diag = VectorXd::Map(task_space["B_T"].as<std::vector<double>>().data(), 6);

        M_T_ = M_T_diag.asDiagonal();
        K_T_ = K_T_diag.asDiagonal();
        B_T_ = B_T_diag.asDiagonal();

        // Load joint space gains
        const auto& joint_space = config["joint_space_gains"];
        Kp_ = VectorXd::Map(joint_space["Kp"].as<std::vector<double>>().data(), arm_model_.nq);
        Kd_ = VectorXd::Map(joint_space["Kd"].as<std::vector<double>>().data(), arm_model_.nq);

    }

    MatrixXd RobotData::getMT()
    {
        return M_T_;
    }

    MatrixXd RobotData::getKT()
    {
        return K_T_;
    }

    MatrixXd RobotData::getBT()
    {
        return B_T_;
    }

    VectorXd RobotData::getKp()
    {
        return Kp_;
    }
    
    VectorXd RobotData::getKd()
    {
        return Kd_;
    }

    bool RobotData::updateState(const VectorXd& pos, const VectorXd& vel, const VectorXd& tau)
    {
        const size_t N = wheel_names_.size();
        const size_t M = joint_names_.size();
        // --- extract the first N entries for the wheels ---
        VectorXd wheel_pose      = pos.segment(0, N);
        VectorXd wheel_vel       = vel.segment(0, N);
        VectorXd wheel_torque    = tau.segment(0, N);

        // now align them by name
        VectorXd wheel_pose_aligned   = VectorXd::Zero(N);
        VectorXd wheel_vel_aligned    = VectorXd::Zero(N);
        VectorXd wheel_torque_aligned = VectorXd::Zero(N);

        auto name_to_index = [&](const std::string &name) -> std::optional<size_t>
        {
            bool f = name.find("front") != std::string::npos;
            bool r = name.find("rear" ) != std::string::npos;
            bool l = name.find("left" ) != std::string::npos;
            bool t = name.find("right") != std::string::npos;
            if (name.find("wheel") == std::string::npos) return std::nullopt;
            if (!( (f ^ r) && (l ^ t) )) return std::nullopt;
            size_t row = f ? 0 : 1;
            size_t col = l ? 0 : 1;
            return row * 2 + col;  // 0..3
        };

        for (size_t i = 0; i < N; ++i) {
        if (auto idx = name_to_index(wheel_names_[i])) {
            wheel_pose_aligned  [i] = wheel_pose[*idx];
            wheel_vel_aligned   [i] = wheel_vel[*idx];
            wheel_torque_aligned[i] = wheel_torque[*idx];
        }
        }

        wheel_pose_   = std::move(wheel_pose_aligned);
        wheel_vel_    = std::move(wheel_vel_aligned);
        wheel_torque_ = std::move(wheel_torque_aligned);

        VectorXd q     = pos.segment  (N+1, M);
        VectorXd qdot  = vel.segment  (N+1, M);
        VectorXd tau_j = tau.segment  (N+1, M);

        q_    = q;
        qdot_ = qdot;
        tau_  = tau_j;

        if (!updateKinematics(q_, qdot_))    return false;
        if (!updateDynamics (q_, qdot_))    return false;
        return true;
    }

    
    VectorXd RobotData::getq()
    {
        return q_;
    }

    VectorXd RobotData::getqdot()
    {
        return qdot_;
    }
    
    VectorXd RobotData::gettau()
    {
        return tau_;
    }

    VectorXd RobotData::getWheelPose()
    {
        return wheel_pose_;
    }
    VectorXd RobotData::getWheelVel()
    {
        return wheel_vel_;
    }

    bool RobotData::updateKinematics(const VectorXd& q, const VectorXd& qdot)
    {
        if(q.size() != arm_model_.nq)
        {
            std::cerr << "updateKinematics Error: size of q " << q.size() << " is not equal to model.nq size: " << arm_model_.nq << std::endl;
            return false;
        }
        if(qdot.size() != arm_model_.nv)
        {
            std::cerr << "updateKinematics Error: size of qdot " << qdot.size() << " is not equal to model.nv size: " << arm_model_.nv << std::endl;
            return false;
        }
        
        pinocchio::FrameIndex link_index = arm_model_.getFrameId(ee_name_);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << ee_name_ << " not found in URDF." << std::endl;
            return false;
        }

        pinocchio::computeJointJacobians(arm_model_, arm_data_, q);
        pinocchio::computeJointJacobiansTimeVariation(arm_model_, arm_data_, q, qdot);
        x_ = getPose(ee_name_);
        J_ = getJacobian(ee_name_);
        xdot_ = J_ * qdot;
        Jdot_ = getJacobianTimeVariation();

        return true;
    }

    bool RobotData::updateDynamics(const VectorXd& q, const VectorXd& qdot)
    {
        if(q.size() != arm_model_.nq)
        {
            std::cerr << "updateDynamics Error: size of q " << q.size() << " is not equal to model.nq size: " << arm_model_.nq << std::endl;
            return false;
        }
        if(qdot.size() != arm_model_.nv)
        {
            std::cerr << "updateDynamics Error: size of qdot " << qdot.size() << " is not equal to model.nv size: " << arm_model_.nv << std::endl;
            return false;
        }
        pinocchio::crba(arm_model_, arm_data_, q);
        pinocchio::computeGeneralizedGravity(arm_model_, arm_data_, q);
        pinocchio::nonLinearEffects(arm_model_, arm_data_, q, qdot);

        // update joint space dynamics
        M_ = arm_data_.M;
        M_ = M_.selfadjointView<Upper>();  // Only upper triangular part of M_ is computed by pinocchio::crba
        M_inv_ = M_.inverse();
        g_ = arm_data_.g;
        NLE_ = arm_data_.nle;
        c_ = NLE_ - g_;

        // update task space dynamic
        M_ee_inv_ = J_ * M_inv_ * J_.transpose();
        M_ee_ = M_ee_inv_.inverse();
        c_ee_ = M_ee_ * (J_ * M_ * c_ - Jdot_ * qdot_);
        g_ee_ = M_ee_ * J_ * M_inv_ * g_;
        NLE_ee_ = c_ee_ + g_ee_;

        return true;
    }

    Matrix4d RobotData::computePose(const VectorXd& q, const std::string& link_name)
    {
        assert(q.size() == arm_model_.nq);

        pinocchio::FrameIndex link_index = arm_model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return Matrix4d::Identity();
        }
        pinocchio::Data tmp_data;
        pinocchio::framesForwardKinematics(arm_model_, tmp_data, q);
        Matrix4d link_pose = tmp_data.oMf[link_index].toHomogeneousMatrix();

        return link_pose;
    }

    Matrix4d RobotData::getPose(const std::string& link_name)
    {
        pinocchio::FrameIndex link_index = arm_model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return Matrix4d::Identity();
        }
        Matrix4d link_pose = arm_data_.oMf[link_index].toHomogeneousMatrix();

        return link_pose;
    }

    MatrixXd RobotData::computeJacobian(const VectorXd& q, const std::string& link_name)
    {
        assert(q.size() == arm_model_.nq);

        pinocchio::FrameIndex link_index = arm_model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return MatrixXd::Zero(6, arm_model_.nv);
        }
        MatrixXd J;
        J.setZero(6, arm_model_.nv);
        pinocchio::Data tmp_data;
        pinocchio::computeFrameJacobian(arm_model_, tmp_data, q, link_index, J);

        return J;
    }

    MatrixXd RobotData::getJacobian(const std::string& link_name)
    {
        pinocchio::FrameIndex link_index = arm_model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return MatrixXd::Zero(6, arm_model_.nv);
        }
        MatrixXd J = pinocchio::getFrameJacobian(arm_model_, arm_data_, link_index, pinocchio::ReferenceFrame::WORLD);

        return J;
    }

    MatrixXd RobotData::computeJacobianTimeVariation(const VectorXd& q, const VectorXd& qdot, const std::string& link_name)
    {
        assert(q.size() == arm_model_.nq);
        assert(qdot.size() == arm_model_.nv);

        pinocchio::FrameIndex link_index = arm_model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return MatrixXd::Zero(6, arm_model_.nv);
        }
        MatrixXd Jdot;
        Jdot.setZero(6, arm_model_.nv);
        pinocchio::Data tmp_data;
        pinocchio::computeJointJacobiansTimeVariation(arm_model_, tmp_data, q, qdot_);
        pinocchio::getFrameJacobianTimeVariation(arm_model_, tmp_data, link_index, pinocchio::ReferenceFrame::WORLD, Jdot);

        return Jdot;
    }

    MatrixXd RobotData::getJacobianTimeVariation(const std::string& link_name)
    {
        pinocchio::FrameIndex link_index = arm_model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return MatrixXd::Zero(6, arm_model_.nv);
        }
        MatrixXd Jdot;
        Jdot.setZero(6, arm_model_.nv);
        pinocchio::getFrameJacobianTimeVariation(arm_model_, arm_data_, link_index, pinocchio::ReferenceFrame::WORLD, Jdot);

        return Jdot;
    }

    VectorXd RobotData::computeVelocity(const VectorXd& q, const VectorXd& qdot, const std::string& link_name)
    {
        assert(q.size() == arm_model_.nq);
        assert(qdot.size() == arm_model_.nv);

        MatrixXd J = computeJacobian(q, link_name);
        
        return J * qdot;
    }

    VectorXd RobotData::getVelocity(const std::string& link_name)
    {
        MatrixXd J = getJacobian(link_name);

        return J * qdot_;
    }

    MatrixXd RobotData::computeMassMatrix(const VectorXd& q)
    {
        assert(q.size() == arm_model_.nq);
        
        pinocchio::Data tmp_data;
        pinocchio::crba(arm_model_, tmp_data, q);
        tmp_data.M = tmp_data.M.selfadjointView<Upper>();  // Only upper triangular part of M_ is computed by pinocchio::crba

        return tmp_data.M;
    }

    MatrixXd RobotData::getMassMatrix()
    {
        return M_;
    }

    VectorXd RobotData::computeCoriolis(const VectorXd& q, const VectorXd& qdot)
    {
        assert(q.size() == arm_model_.nq);
        assert(qdot.size() == arm_model_.nv);
        
        pinocchio::Data tmp_data;
        pinocchio::computeCoriolisMatrix(arm_model_, tmp_data, q, qdot);

        return tmp_data.C * qdot;
    }

    VectorXd RobotData::getCoriolis()
    {
        return c_;
    }

    VectorXd RobotData::computeGravity(const VectorXd& q)
    {
        assert(q.size() == arm_model_.nq);
        
        pinocchio::Data tmp_data;
        pinocchio::computeGeneralizedGravity(arm_model_, tmp_data, q);

        return tmp_data.g;
    }

    VectorXd RobotData::getGravity()
    {
        return g_;
    }

    VectorXd RobotData::computeNonlinearEffects(const VectorXd& q, const VectorXd& qdot)
    {
        assert(q.size() == arm_model_.nq);
        assert(qdot.size() == arm_model_.nv);

        pinocchio::Data tmp_data;
        pinocchio::nonLinearEffects(arm_model_, tmp_data, q, qdot);

        return tmp_data.nle;
    }

    VectorXd RobotData::getNonlinearEffects()
    {
        return NLE_;
    }

    MatrixXd RobotData::computeTaskMassMatrix(const VectorXd& q, const std::string& link_name)
    {
        assert(q.size() == arm_model_.nq);
        pinocchio::FrameIndex link_index = arm_model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return MatrixXd::Zero(6, 6);
        }

        MatrixXd M = computeMassMatrix(q);
        MatrixXd J = computeJacobian(q, link_name);

        return (J * M.inverse() * J.transpose()).inverse();
    }

    MatrixXd RobotData::getTaskMassMatrix(const std::string& link_name)
    {
        pinocchio::FrameIndex link_index = arm_model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return MatrixXd::Zero(6, 6);
        }

        MatrixXd J = getJacobian(link_name);

        return (J * M_.inverse() * J.transpose()).inverse();
    }

    VectorXd RobotData::computeTaskCoriolis(const VectorXd& q, const VectorXd& qdot, const std::string& link_name)
    {
        assert(q.size() == arm_model_.nq);
        assert(qdot.size() == arm_model_.nv);
        pinocchio::FrameIndex link_index = arm_model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return VectorXd::Zero(6);
        }

        MatrixXd M = computeMassMatrix(q);
        MatrixXd J = computeJacobian(q, link_name);
        MatrixXd M_task = computeTaskMassMatrix(q, link_name);
        VectorXd c = computeCoriolis(q, qdot);
        MatrixXd Jdot = computeJacobianTimeVariation(q, qdot, link_name);

        return M_task * J * M * c - M_task * Jdot * qdot;
    }

    VectorXd RobotData::getTaskCoriolis(const std::string& link_name)
    {
        pinocchio::FrameIndex link_index = arm_model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return VectorXd::Zero(6);
        }

        MatrixXd J = getJacobian(link_name);
        MatrixXd M_task = getTaskMassMatrix(link_name);
        MatrixXd Jdot = getJacobianTimeVariation(link_name);

        return M_task * J * M_ * c_ - M_task * Jdot * qdot_;
    }

    VectorXd RobotData::computeTaskGravity(const VectorXd& q, const std::string& link_name)
    {
        assert(q.size() == arm_model_.nq);
        pinocchio::FrameIndex link_index = arm_model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return VectorXd::Zero(6);
        }

        MatrixXd M = computeMassMatrix(q);
        MatrixXd J = computeJacobian(q, link_name);
        MatrixXd M_task = computeTaskMassMatrix(q, link_name);
        VectorXd g = computeGravity(q);

        return M_task * J * M.inverse() * g;
    }

    VectorXd RobotData::getTaskGravity(const std::string& link_name)
    {
        pinocchio::FrameIndex link_index = arm_model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return VectorXd::Zero(6);
        }

        MatrixXd J = getJacobian(link_name);
        MatrixXd M_task = getTaskMassMatrix(link_name);

        return M_task * J * M_.inverse() * g_;
    }

    VectorXd RobotData::computeTaskNonlinearEffects(const VectorXd& q, const VectorXd& qdot, const std::string& link_name)
    {
        assert(q.size() == arm_model_.nq);
        assert(qdot.size() == arm_model_.nv);
        pinocchio::FrameIndex link_index = arm_model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return VectorXd::Zero(6);
        }

        VectorXd c_task = computeTaskCoriolis(q, qdot, link_name);
        VectorXd g_task = computeTaskGravity(q, link_name);

        return c_task + g_task;
    }

    VectorXd RobotData::getTaskNonlinearEffects(const std::string& link_name)
    {
        pinocchio::FrameIndex link_index = arm_model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return VectorXd::Zero(6);
        }

        VectorXd c_task = getTaskCoriolis(link_name);
        VectorXd g_task = getTaskGravity(link_name);

        return c_task + g_task;
    }

    VectorXd RobotData::getWrench(const std::string& link_name)
    {
        pinocchio::FrameIndex link_index = arm_model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return VectorXd::Zero(6);
        }

        MatrixXd J_hand = RobotData::getJacobian(link_name);
        MatrixXd J_bar = M_inv_ * J_hand.transpose() * M_ee_;
        VectorXd tau_ext = tau_ - NLE_;
        VectorXd wrench_ee = J_bar.transpose() * tau_ext;

        return wrench_ee;
    }

}