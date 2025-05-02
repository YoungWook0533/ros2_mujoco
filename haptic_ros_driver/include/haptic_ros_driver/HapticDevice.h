#ifndef HAPTIC_ROS_DRIVER__HAPTICDEVICE_HPP_
#define HAPTIC_ROS_DRIVER__HAPTICDEVICE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <Eigen/Dense>
#include <thread>
#include <mutex>
#include <atomic>

// low-level Haptic API
#include "haptic_ros_driver/dhdc.h"

namespace haptic_ros_driver
{

class HapticDevice : public rclcpp::Node
{
public:
  HapticDevice(double loop_hz = 1000.0, bool set_force = true);
  ~HapticDevice() override;

  /// Start background thread + spinning
  void Start();

private:
  void RegisterCallback();
  void PublishHapticData();
  void GetHapticDataRun();
  void ForceCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);
  void WrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  void SetForce(const Eigen::Vector3d &force);
  void VerifyForceLimit(Eigen::Vector3d &force);
  void ApplyReturnToOriginForce();

  // publishers & subscribers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr ori_encoder_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr button_state_pub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr force_sub_;

  // timing
  double loop_hz_;
  rclcpp::Rate loop_rate_;

  // device thread
  std::thread dev_op_thread_;
  std::atomic<bool> keep_alive_;

  // device state
  int  device_count_;
  int  dev_id_;
  bool set_force_;
  bool device_enabled_;

  // data protection
  std::mutex val_lock_;
  bool force_released_;

  // haptic state vectors
  Eigen::Vector3d force_limit_;
  Eigen::Vector3d filtered_force_feedback_;
  Eigen::Vector3d position_;
  Eigen::Matrix3d orientation_;
  Eigen::Vector3d ori_encoder_;
  Eigen::Vector3d force_;
  Eigen::Vector3d lin_vel_;
  Eigen::Vector3d ang_vel_;
  bool button0_state_;
};

}  // namespace haptic_ros_driver

#endif  // HAPTIC_ROS_DRIVER__HAPTICDEVICE_HPP_
