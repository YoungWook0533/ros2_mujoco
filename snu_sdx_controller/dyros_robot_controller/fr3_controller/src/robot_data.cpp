#include "robot_data.h"
#include <yaml-cpp/yaml.h>


namespace FR3Controller
{
    RobotData::RobotData(const std::string& urdf_path, const std::string& yaml_path)
    {
        std::ifstream file(urdf_path);
        if (!file.good()) std::cout << "URDF file does not exist! : " << urdf_path << std::endl;
    
        pinocchio::urdf::buildModel(urdf_path, model_);
        data_ = pinocchio::Data(model_);
                
        joint_names_ = model_.names;
        joint_names_.erase(joint_names_.begin());  // Remove the first element "universe_joint"

        // Initialize joint space state
        q_ = VectorXd::Zero(model_.nq);
        qdot_ = VectorXd::Zero(model_.nq);
        qddot_ = VectorXd::Zero(model_.nq);
        // qdot_prev_ = VectorXd::Zero(model_.nq);
        tau_ = VectorXd::Zero(model_.nq);

        // Initialize task space state
        x_ = Matrix4d::Identity();
        xdot_ = VectorXd::Zero(6);
        J_ = MatrixXd::Zero(6, model_.nv);
        Jdot_ = MatrixXd::Zero(6, model_.nv);

        // Initialize joint space dynamics
        M_ = MatrixXd::Zero(model_.nq, model_.nv);
        M_inv_ = MatrixXd::Zero(model_.nq, model_.nv);
        g_ = VectorXd::Zero(model_.nq);       
        c_ = VectorXd::Zero(model_.nq);       
        NLE_ = VectorXd::Zero(model_.nq);       

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
        Kp_.resize(model_.nq);
        Kd_.resize(model_.nq);

        p_hat_ = VectorXd::Zero(model_.nq);
        p0_ = VectorXd::Zero(model_.nq);
        p_ = VectorXd::Zero(model_.nq);
        M_prev_ = MatrixXd::Zero(model_.nq, model_.nv);
        is_initialized_ = false;
        K0_ = 40 * MatrixXd::Identity(model_.nq, model_.nq);

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
        Kp_ = VectorXd::Map(joint_space["Kp"].as<std::vector<double>>().data(), model_.nq);
        Kd_ = VectorXd::Map(joint_space["Kd"].as<std::vector<double>>().data(), model_.nq);

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

    bool RobotData::updateState(const VectorXd& q, const VectorXd& qdot, const VectorXd& qddot, const VectorXd& tau)
    {
        q_ = q;
        qdot_ = qdot;
        qddot_ = qddot;
        // qddot_ = (qdot_ - qdot_prev_) / 0.001;
        // qdot_prev_ = qdot_;
        tau_ = tau;
        if(!updateKinematics(q_, qdot_)) return false;
        if(!updateDynamics(q_, qdot_)) return false;
        runMOB(0.001);
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

    bool RobotData::updateKinematics(const VectorXd& q, const VectorXd& qdot)
    {
        if(q.size() != model_.nq)
        {
            std::cerr << "updateDynamics Error: size of q " << q.size() << " is not equal to model.nq size: " << model_.nq << std::endl;
            return false;
        }
        if(qdot.size() != model_.nv)
        {
            std::cerr << "updateDynamics Error: size of qdot " << qdot.size() << " is not equal to model.nv size: " << model_.nv << std::endl;
            return false;
        }
        
        pinocchio::FrameIndex link_index = model_.getFrameId(ee_name_);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << ee_name_ << " not found in URDF." << std::endl;
            return false;
        }

        pinocchio::computeJointJacobians(model_, data_, q);
        pinocchio::computeJointJacobiansTimeVariation(model_, data_, q, qdot);
        x_ = getPose(ee_name_);
        J_ = getJacobian(ee_name_);
        xdot_ = J_ * qdot;
        Jdot_ = getJacobianTimeVariation();

        return true;
    }

    bool RobotData::updateDynamics(const VectorXd& q, const VectorXd& qdot)
    {
        if(q.size() != model_.nq)
        {
            std::cerr << "updateDynamics Error: size of q " << q.size() << " is not equal to model.nq size: " << model_.nq << std::endl;
            return false;
        }
        if(qdot.size() != model_.nv)
        {
            std::cerr << "updateDynamics Error: size of qdot " << qdot.size() << " is not equal to model.nv size: " << model_.nv << std::endl;
            return false;
        }
        pinocchio::crba(model_, data_, q);
        pinocchio::computeGeneralizedGravity(model_, data_, q);
        pinocchio::nonLinearEffects(model_, data_, q, qdot);

        // update joint space dynamics
        M_ = data_.M;
        M_ = M_.selfadjointView<Upper>();  // Only upper triangular part of M_ is computed by pinocchio::crba
        M_inv_ = M_.inverse();
        g_ = data_.g;
        NLE_ = data_.nle;
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
        assert(q.size() == model_.nq);

        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return Matrix4d::Identity();
        }
        pinocchio::Data tmp_data;
        pinocchio::framesForwardKinematics(model_, tmp_data, q);
        Matrix4d link_pose = tmp_data.oMf[link_index].toHomogeneousMatrix();

        return link_pose;
    }

    Matrix4d RobotData::getPose(const std::string& link_name)
    {
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return Matrix4d::Identity();
        }
        Matrix4d link_pose = data_.oMf[link_index].toHomogeneousMatrix();

        return link_pose;
    }

    MatrixXd RobotData::computeJacobian(const VectorXd& q, const std::string& link_name)
    {
        assert(q.size() == model_.nq);

        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return MatrixXd::Zero(6, model_.nv);
        }
        MatrixXd J;
        J.setZero(6, model_.nv);
        pinocchio::Data tmp_data;
        pinocchio::computeFrameJacobian(model_, tmp_data, q, link_index, J);

        return J;
    }

    MatrixXd RobotData::getJacobian(const std::string& link_name)
    {
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return MatrixXd::Zero(6, model_.nv);
        }
        MatrixXd J = pinocchio::getFrameJacobian(model_, data_, link_index, pinocchio::ReferenceFrame::WORLD);

        return J;
    }

    MatrixXd RobotData::computeJacobianTimeVariation(const VectorXd& q, const VectorXd& qdot, const std::string& link_name)
    {
        assert(q.size() == model_.nq);
        assert(qdot.size() == model_.nv);

        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return MatrixXd::Zero(6, model_.nv);
        }
        MatrixXd Jdot;
        Jdot.setZero(6, model_.nv);
        pinocchio::Data tmp_data;
        pinocchio::computeJointJacobiansTimeVariation(model_, tmp_data, q, qdot_);
        pinocchio::getFrameJacobianTimeVariation(model_, tmp_data, link_index, pinocchio::ReferenceFrame::WORLD, Jdot);

        return Jdot;
    }

    MatrixXd RobotData::getJacobianTimeVariation(const std::string& link_name)
    {
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return MatrixXd::Zero(6, model_.nv);
        }
        MatrixXd Jdot;
        Jdot.setZero(6, model_.nv);
        pinocchio::getFrameJacobianTimeVariation(model_, data_, link_index, pinocchio::ReferenceFrame::WORLD, Jdot);

        return Jdot;
    }

    VectorXd RobotData::computeVelocity(const VectorXd& q, const VectorXd& qdot, const std::string& link_name)
    {
        assert(q.size() == model_.nq);
        assert(qdot.size() == model_.nv);

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
        assert(q.size() == model_.nq);
        
        pinocchio::Data tmp_data;
        pinocchio::crba(model_, tmp_data, q);
        tmp_data.M = tmp_data.M.selfadjointView<Upper>();  // Only upper triangular part of M_ is computed by pinocchio::crba

        return tmp_data.M;
    }

    MatrixXd RobotData::getMassMatrix()
    {
        return M_;
    }

    VectorXd RobotData::computeCoriolis(const VectorXd& q, const VectorXd& qdot)
    {
        assert(q.size() == model_.nq);
        assert(qdot.size() == model_.nv);
        
        pinocchio::Data tmp_data;
        pinocchio::computeCoriolisMatrix(model_, tmp_data, q, qdot);

        return tmp_data.C * qdot;
    }

    VectorXd RobotData::getCoriolis()
    {
        return c_;
    }

    VectorXd RobotData::computeGravity(const VectorXd& q)
    {
        assert(q.size() == model_.nq);
        
        pinocchio::Data tmp_data;
        pinocchio::computeGeneralizedGravity(model_, tmp_data, q);

        return tmp_data.g;
    }

    VectorXd RobotData::getGravity()
    {
        return g_;
    }

    VectorXd RobotData::computeNonlinearEffects(const VectorXd& q, const VectorXd& qdot)
    {
        assert(q.size() == model_.nq);
        assert(qdot.size() == model_.nv);

        pinocchio::Data tmp_data;
        pinocchio::nonLinearEffects(model_, tmp_data, q, qdot);

        return tmp_data.nle;
    }

    VectorXd RobotData::getNonlinearEffects()
    {
        return NLE_;
    }

    MatrixXd RobotData::computeTaskMassMatrix(const VectorXd& q, const std::string& link_name)
    {
        assert(q.size() == model_.nq);
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
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
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
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
        assert(q.size() == model_.nq);
        assert(qdot.size() == model_.nv);
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
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
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
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
        assert(q.size() == model_.nq);
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
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
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
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
        assert(q.size() == model_.nq);
        assert(qdot.size() == model_.nv);
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
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
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return VectorXd::Zero(6);
        }

        VectorXd c_task = getTaskCoriolis(link_name);
        VectorXd g_task = getTaskGravity(link_name);

        return c_task + g_task;
    }

    void RobotData::runMOB(double dt)
    {
        MatrixXd M_dot = (M_ - M_prev_) / dt;
        M_prev_ = M_;

        VectorXd C_T_qdot = M_dot * qdot_ - c_;
        VectorXd Beta = g_ - C_T_qdot;

        p_ = M_ * qdot_;
        if (!is_initialized_) {
            p0_ = p_;
            is_initialized_ = true;
        }

        r_ = K0_ * (p_ - p0_ - p_hat_);
        VectorXd pdot_hat = tau_ - Beta + r_;
        p_hat_ += pdot_hat * dt;
    }

    VectorXd RobotData::getWrench(const std::string& link_name)
    {
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return VectorXd::Zero(6);
        }

        MatrixXd J_hand = RobotData::getJacobian(link_name);
        MatrixXd J_bar = M_inv_ * J_hand.transpose() * M_ee_;
        VectorXd tau_model = M_ * qddot_ + c_ + g_;
        VectorXd tau_ext = tau_ - tau_model;
        VectorXd wrench_ee = J_bar.transpose() * tau_ext;
        
        return wrench_ee;
    }
}