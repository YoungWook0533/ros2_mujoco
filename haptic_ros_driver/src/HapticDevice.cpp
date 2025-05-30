#include "haptic_ros_driver/HapticDevice.h"
#include <chrono>
#include <algorithm>

using namespace std::chrono_literals;

namespace haptic_ros_driver
{

  HapticDevice::HapticDevice(double loop_hz, bool set_force)
  : Node("haptic_device"),
    loop_hz_(loop_hz),
    loop_rate_(loop_hz_),
    keep_alive_(false),
    device_count_(0),
    dev_id_(-1),
    set_force_(set_force),
    device_enabled_(false),
    force_released_(true),
    force_limit_(10.0, 10.0, 10.0),
    filtered_force_feedback_ (Eigen::Vector3d::Zero()),
    position_(Eigen::Vector3d::Zero()),
    orientation_(Eigen::Matrix3d::Identity()),
    ori_encoder_(Eigen::Vector3d::Zero()),
    force_(Eigen::Vector3d::Zero()),
    lin_vel_(Eigen::Vector3d::Zero()),
    ang_vel_(Eigen::Vector3d::Zero()),
    button0_state_(false)
  {
    device_count_ = dhdGetDeviceCount();
    if (device_count_ >= 1) {
      dev_id_ = dhdOpenID(0);
      if (dev_id_ < 0) {
        RCLCPP_WARN(this->get_logger(), "Failed to open haptic device: %s", dhdErrorGetLastStr());
      } else {
        device_enabled_ = true;
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "No haptic device found: %s", dhdErrorGetLastStr());
    }

    if (device_enabled_ && set_force_) {
      if (dhdEnableForce(DHD_ON) != DHD_NO_ERROR) {
        RCLCPP_ERROR(this->get_logger(), "Failed to enable force: %s", dhdErrorGetLastStr());
        device_enabled_ = false;
      }
    }

    RegisterCallback();
  }

  HapticDevice::~HapticDevice()
  {
    keep_alive_ = false;
    if (dev_op_thread_.joinable()) {
      dev_op_thread_.join();
    }

    if (dhdClose() < 0) {
      RCLCPP_WARN(this->get_logger(), "Failed to close haptic device: %s", dhdErrorGetLastStr());
      dhdSleep(2.0);
    }
  }

  void HapticDevice::RegisterCallback()
  {
    pose_pub_          = create_publisher<geometry_msgs::msg::PoseStamped>("/haptic/pose", 1);
    ori_encoder_pub_   = create_publisher<std_msgs::msg::Float32MultiArray>("/haptic/encoder_orientation", 1);
    twist_pub_         = create_publisher<geometry_msgs::msg::Twist>("/haptic/twist", 1);
    button_state_pub_  = create_publisher<std_msgs::msg::Int8MultiArray>("/haptic/button_state", 1);

    force_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
      "/haptic/force", 1,
      std::bind(&HapticDevice::ForceCallback, this, std::placeholders::_1));

    wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/wrench", 1,
      std::bind(&HapticDevice::WrenchCallback, this, std::placeholders::_1));
  }

  void HapticDevice::Start()
  {
    if (!device_enabled_) {
      RCLCPP_ERROR(this->get_logger(), "Haptic device not enabled, aborting Start().");
      return;
    }

    keep_alive_ = true;
    dev_op_thread_ = std::thread(&HapticDevice::GetHapticDataRun, this);

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(this->get_node_base_interface());
    exec.spin();

    keep_alive_ = false;
    if (dev_op_thread_.joinable()) {
      dev_op_thread_.join();
    }
  }

  void HapticDevice::GetHapticDataRun()
  {
    double cur_pos[3], cur_ori[3][3], cur_enc[3], cur_lin[3], cur_ang[3];

    while (rclcpp::ok() && keep_alive_) {
      if (dev_id_ >= 0) {
        dhdGetPosition(cur_pos, cur_pos+1, cur_pos+2);
        position_ = Eigen::Vector3d(cur_pos[0], cur_pos[1], cur_pos[2]);

        dhdGetOrientationFrame(cur_ori);
        for (int i=0; i<3; ++i)
          for (int j=0; j<3; ++j)
            orientation_(i,j) = cur_ori[i][j];

        dhdGetOrientationRad(cur_enc, cur_enc+1, cur_enc+2);
        ori_encoder_ = Eigen::Vector3d(cur_enc[0], cur_enc[1], cur_enc[2]);

        dhdGetLinearVelocity(cur_lin, cur_lin+1, cur_lin+2);
        lin_vel_ = Eigen::Vector3d(cur_lin[0], cur_lin[1], cur_lin[2]);

        dhdGetAngularVelocityRad(cur_ang, cur_ang+1, cur_ang+2);
        ang_vel_ = Eigen::Vector3d(cur_ang[0], cur_ang[1], cur_ang[2]);

        button0_state_ = (dhdGetButton(0, dev_id_) != 0);
      }

      PublishHapticData();
      ApplyReturnToOriginForce();

      if (set_force_) {
        std::lock_guard<std::mutex> lock(val_lock_);
        double ff[3] = { force_.x(), force_.y(), force_.z() };
        if (dhdSetForce(ff[0], ff[1], ff[2]) != DHD_NO_ERROR) {
          RCLCPP_WARN(this->get_logger(), "Failed to set force: %s", dhdErrorGetLastStr());
        }
      }

      loop_rate_.sleep();
    }
  }

  void HapticDevice::PublishHapticData()
  {
    // pose
    auto pose = geometry_msgs::msg::PoseStamped();
    pose.header.frame_id = get_name();
    pose.header.stamp    = now();
    pose.pose.position.x = position_.x();
    pose.pose.position.y = position_.y();
    pose.pose.position.z = position_.z();
    Eigen::Quaterniond q(orientation_);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    pose_pub_->publish(pose);

    // encoder
    auto enc = std_msgs::msg::Float32MultiArray();
    enc.data = { static_cast<float>(ori_encoder_.x()),
                static_cast<float>(ori_encoder_.y()),
                static_cast<float>(ori_encoder_.z()) };
    ori_encoder_pub_->publish(enc);

    // twist
    auto tw = geometry_msgs::msg::Twist();
    tw.linear.x  = lin_vel_.x();
    tw.linear.y  = lin_vel_.y();
    tw.linear.z  = lin_vel_.z();
    tw.angular.x = ang_vel_.x();
    tw.angular.y = ang_vel_.y();
    tw.angular.z = ang_vel_.z();
    twist_pub_->publish(tw);

    // button
    auto btn = std_msgs::msg::Int8MultiArray();
    btn.data = { static_cast<int8_t>(button0_state_) };
    button_state_pub_->publish(btn);
  }

  void HapticDevice::ForceCallback(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    Eigen::Vector3d f(msg->x, msg->y, msg->z);
    SetForce(f);
  }

  void HapticDevice::WrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    Eigen::Vector3d f(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
    filtered_force_feedback_ = 0.03045 * f + (1.0 - 0.03045) * filtered_force_feedback_;
    for (int i = 0; i < 3; ++i) {
      filtered_force_feedback_[i] =
        std::clamp(filtered_force_feedback_[i], -5.0, 5.0);
    }
    for (size_t i = 0; i < 3; i++)
    {
      std::cout<<filtered_force_feedback_[i]<<", ";
    }
    std::cout<<std::endl;
    SetForce(0.1 * filtered_force_feedback_);
  }

  void HapticDevice::SetForce(const Eigen::Vector3d &f_in)
  {
    if (!set_force_) return;
    std::lock_guard<std::mutex> lock(val_lock_);
    Eigen::Vector3d f = f_in;
    VerifyForceLimit(f);
    force_ = f;
    force_released_ = false;
  }

  void HapticDevice::VerifyForceLimit(Eigen::Vector3d &f)
  {
    for (int i = 0; i < 3; ++i) {
      f[i] = std::clamp(f[i], -force_limit_[i], force_limit_[i]);
    }
  }

  void HapticDevice::ApplyReturnToOriginForce()
  {
    if (!set_force_) return;
    std::lock_guard<std::mutex> lock(val_lock_);
    double p_gain = 100.0, d_gain = 5.0;
    Eigen::Vector3d err = -position_;
    if (err.norm() < 0.01) p_gain *= 5.0;
    force_ = p_gain * err - d_gain * lin_vel_;
  }

} 
