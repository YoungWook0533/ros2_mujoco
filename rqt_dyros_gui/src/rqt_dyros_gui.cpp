// rqt_dyros_gui.cpp

#include "rqt_dyros_gui/rqt_dyros_gui.h"
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/msg/int32.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <builtin_interfaces/msg/duration.hpp>

namespace rqt_dyros_gui
{

RqtDyrosPlugin::RqtDyrosPlugin()
  : rqt_gui_cpp::Plugin()
  , widget_(nullptr)
{
  setObjectName("RqtDyrosPlugin");
}

void RqtDyrosPlugin::initPlugin(qt_gui_cpp::PluginContext &context)
{
  // --- ROS2 publishers ---
  mode_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/key_input", 10);
  ee_pub_   = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/ee_commands", 10);

  // --- load the .ui ---
  QStringList argv = context.argv();
  widget_ = new QDialog();
  ui_.setupUi(widget_);
  context.addWidget(widget_);

  // --- connect the four mode buttons ---
  connect(ui_.pushButton,   &QPushButton::clicked, this, [this]() { publishMode(1, "init"); });
  connect(ui_.pushButton_2, &QPushButton::clicked, this, [this]() { publishMode(2, "home"); });
  connect(ui_.pushButton_3, &QPushButton::clicked, this, [this]() { publishMode(3, "cartesian"); });
  connect(ui_.pushButton_4, &QPushButton::clicked, this, [this]() { publishMode(4, "teleop"); });

  // --- connect the Send button ---
  connect(ui_.pushButton_5, &QPushButton::clicked, this, &RqtDyrosPlugin::sendEECommand);
}

void RqtDyrosPlugin::publishMode(int mode, const QString &label)
{
  std_msgs::msg::Int32 msg;
  msg.data = mode;
  mode_pub_->publish(msg);
  ui_.lineEdit->setText(label);
}

void RqtDyrosPlugin::sendEECommand()
{
  // read the 6 EE‐command fields + duration
  bool ok = false;
  float x   = ui_.lineEdit_1 ->text().toFloat(&ok);
  float y   = ui_.lineEdit_2 ->text().toFloat(&ok);
  float z   = ui_.lineEdit_3 ->text().toFloat(&ok);
  float r   = ui_.lineEdit_4 ->text().toFloat(&ok);
  float p   = ui_.lineEdit_5 ->text().toFloat(&ok);
  float yaw = ui_.lineEdit_6 ->text().toFloat(&ok);
  float dur = ui_.lineEdit_7 ->text().toFloat(&ok);

  trajectory_msgs::msg::JointTrajectory traj;
  trajectory_msgs::msg::JointTrajectoryPoint pt;
  pt.positions  = {x, y, z, r, p, yaw};
  pt.velocities = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

  // convert dur (seconds.fraction) into ROS Duration
  int32_t sec  = static_cast<int32_t>(std::floor(dur));
  int32_t nano = static_cast<int32_t>((dur - sec) * 1e9);
  builtin_interfaces::msg::Duration ros_dur;
  ros_dur.sec    = sec;
  ros_dur.nanosec= nano;
  pt.time_from_start = ros_dur;

  traj.points.push_back(pt);
  ee_pub_->publish(traj);
}

void RqtDyrosPlugin::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                  qt_gui_cpp::Settings &instance_settings) const
{
  (void)plugin_settings;
  // e.g. instance_settings.setValue("last_mode", ui_.textBrowser->toPlainText());
}

void RqtDyrosPlugin::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                     const qt_gui_cpp::Settings &instance_settings)
{
  (void)plugin_settings;
  // e.g. ui_.textBrowser->setText(instance_settings.value("last_mode").toString());
}

} // namespace rqt_dyros_gui

PLUGINLIB_EXPORT_CLASS(rqt_dyros_gui::RqtDyrosPlugin, rqt_gui_cpp::Plugin)
