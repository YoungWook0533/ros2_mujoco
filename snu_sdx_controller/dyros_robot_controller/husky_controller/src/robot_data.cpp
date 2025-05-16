#include "robot_data.h"
#include <optional>


namespace HuskyController
{
    RobotData::RobotData(const std::string& urdf_path)
    {
        std::ifstream file(urdf_path);
        if (!file.good()) std::cout << "URDF file does not exist! : " << urdf_path << std::endl;
    
        pinocchio::urdf::buildModel(urdf_path, model_);
        data_ = pinocchio::Data(model_);
                
        wheel_names_ = model_.names;
        wheel_names_.erase(wheel_names_.begin());  // Remove the first element "universe_joint"

        wheel_pose_ = VectorXd::Zero(wheel_names_.size());
        wheel_vel_ = VectorXd::Zero(wheel_names_.size());
        wheel_torque_ = VectorXd::Zero(wheel_names_.size());

    }

    RobotData::~RobotData()
    {

    }

    void RobotData::updateState(const VectorXd& wheel_pose, const VectorXd& wheel_vel, const VectorXd& wheel_torque)
    {
        const size_t N = wheel_names_.size();
        VectorXd wheel_pose_align = VectorXd::Zero(N);
        VectorXd wheel_vel_align = VectorXd::Zero(N);
        VectorXd wheel_torque_align = VectorXd::Zero(N);
    
        // Align wheel states by name
        auto name_to_index = [&](const std::string &name) -> std::optional<size_t>
        {
            // front‐left(0), front‐right(1), rear‐left(2), rear‐right(3)
            bool is_front = (name.find("front") != std::string::npos);
            bool is_left  = (name.find("left")  != std::string::npos);
            bool is_rear  = (name.find("rear")  != std::string::npos);
            bool is_right = (name.find("right") != std::string::npos);
            
            if (name.find("wheel") == std::string::npos) return std::nullopt;
            if (!( (is_front^is_rear) && (is_left^is_right) )) return std::nullopt;
            
            size_t row = is_front ? 0 : 1;     // front→row=0, rear→row=1
            size_t col = is_left  ? 0 : 1;     // left →col=0, right→col=1
            return row*2 + col;                // index in [0..3]
        };
    
        for (size_t i = 0; i < N; ++i)
        {
            auto maybe_idx = name_to_index(wheel_names_[i]);
            if (!maybe_idx) continue;

            size_t idx = *maybe_idx;  // in 0..3
            wheel_pose_align  [i] = wheel_pose[idx];
            wheel_vel_align   [i] = wheel_vel[idx];
            wheel_torque_align[i] = wheel_torque[idx];
        }
    
        wheel_pose_   = std::move(wheel_pose_align);
        wheel_vel_    = std::move(wheel_vel_align);
        wheel_torque_ = std::move(wheel_torque_align);
    }
    

    VectorXd RobotData::getWheelPose()
    {
        return wheel_pose_;
    }
    VectorXd RobotData::getWheelVel()
    {
        return wheel_vel_;
    }

}