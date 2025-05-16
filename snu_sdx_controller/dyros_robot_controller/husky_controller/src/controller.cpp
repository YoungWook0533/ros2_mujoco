#include "controller.h"
#include "robot_data.h"
#include <cmath>
#include <yaml-cpp/yaml.h>

namespace HuskyController
{ 

    Controller::Controller(double dt, RobotData* rd, const std::string& yaml_path)
    {
        dt_ = dt;
        robot_data_ = rd;

        wheel_names_ = robot_data_->getWheelNames();
        loadConfigFromYaml(yaml_path);
    }

    Controller::~Controller()
    {
    }

    // VectorXd Controller::tmpControl()
    // {
    //     return VectorXd::Zero(7);
    // }

    void Controller::loadConfigFromYaml(const std::string& yaml_file) {

        YAML::Node config = YAML::LoadFile(yaml_file);

        // Load task space gains
        const auto& mobile_config = config["mobile_config"];
        base_width_ = mobile_config["base_width"].as<double>();
        wheel_radius_ = mobile_config["wheel_radius"].as<double>();
        lin_vel_limit_ = mobile_config["lin_vel_limit"].as<double>();
        ang_vel_limit_ = mobile_config["ang_vel_limit"].as<double>();
        lin_acc_limit_ = mobile_config["lin_acc_limit"].as<double>();
        ang_acc_limit_ = mobile_config["ang_acc_limit"].as<double>();

        std::cout << "[Controller] Loaded mobile config:\n";
        std::cout << "  base_width: " << base_width_<< "\n";
        std::cout << "  wheel_radius: " << wheel_radius_<< "\n";
        std::cout << "  lin_vel_limit : " << lin_vel_limit_<< "\n";
        std::cout << "  ang_vel_limit: " << ang_vel_limit_<< "\n";
        std::cout << "  lin_acc_limit: " << lin_acc_limit_<< "\n";
        std::cout << "  ang_acc_limit : " << ang_acc_limit_<< "\n";

    }

    VectorXd Controller::IK(const Vector3d& desired_base_velocity)
    {
        // for (int i = 0; i < 3; i++)
        // {
        //     std::cout << desired_base_velocity[i] << " ";
        // }
        // std::cout<<std::endl;
        
        Vector2d base_velocity(desired_base_velocity[0], desired_base_velocity[2]);
        Matrix2d transformation_matrix;
        transformation_matrix << 1,  base_width_ / 2,
                                 1, -base_width_ / 2;
        transformation_matrix /= wheel_radius_;

        Vector2d desired_wheel_vel = transformation_matrix * base_velocity;
        VectorXd desired_wheel_vel_align(wheel_names_.size());

        for (size_t i = 0; i < wheel_names_.size(); ++i) 
        {
            if (wheel_names_[i].find("wheel") != std::string::npos) 
            {
                if (wheel_names_[i].find("right") != std::string::npos) 
                {
                    desired_wheel_vel_align[i] = desired_wheel_vel[0];
                } 
                else if (wheel_names_[i].find("left") != std::string::npos) 
                {
                    desired_wheel_vel_align[i] = desired_wheel_vel[1];
                }
            }
        }
        return desired_wheel_vel_align;
    }

    Vector3d Controller::FK(const VectorXd& current_wheel_vel)
    {
        Vector2d current_wheel_vel_dealign;
        for (size_t i = 0; i < wheel_names_.size(); ++i) 
        {
            if (wheel_names_[i].find("wheel") != std::string::npos) 
            {
                if (wheel_names_[i].find("right") != std::string::npos) 
                {
                    current_wheel_vel_dealign[0] = current_wheel_vel[i];
                } 
                else if (wheel_names_[i].find("left") != std::string::npos) 
                {
                    current_wheel_vel_dealign[1] = current_wheel_vel[i];
                }
            }
        }

        Matrix2d transformation_matrix;
        transformation_matrix << 0.5,              0.5,
                                 1 / base_width_, -1 / base_width_;
        transformation_matrix *= wheel_radius_;

        Vector2d base_velocity = transformation_matrix * current_wheel_vel_dealign;
        // std::cout<<base_velocity[0]<<'0'<<base_velocity[1]<<std::endl;
        return Vector3d(base_velocity[0], 0, base_velocity[1]);
    }

    VectorXd Controller::VelocityCommand(const Vector3d& desired_base_vel, const Vector3d& current_base_vel)
    {
        Vector3d desired_base_acc = (desired_base_vel - current_base_vel) * dt_;

        if (desired_base_acc.head<2>().norm() > lin_acc_limit_) 
        {
            desired_base_acc.head<2>() = lin_acc_limit_ * desired_base_acc.head<2>().normalized();
        }
        if (std::abs(desired_base_acc[2]) > ang_acc_limit_) 
        {
            desired_base_acc[2] = std::copysign(ang_acc_limit_, desired_base_acc[2]);
        }
        Vector3d new_base_vel = current_base_vel + desired_base_acc / dt_;

        if (new_base_vel.head<2>().norm() > lin_vel_limit_) 
        {
            new_base_vel.head<2>() = lin_vel_limit_ * new_base_vel.head<2>().normalized();
        }
        if (std::abs(new_base_vel[2]) > ang_vel_limit_) 
        {
            new_base_vel[2] = std::copysign(ang_vel_limit_, new_base_vel[2]);
        }

        return IK(new_base_vel);
    }
    
}