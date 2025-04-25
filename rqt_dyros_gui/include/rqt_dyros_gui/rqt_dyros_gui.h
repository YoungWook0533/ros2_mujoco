#ifndef RQT_DYROS_H
#define RQT_DYROS_H

#include <rqt_gui_cpp/rqt_gui_cpp/plugin.h>
#include <ui_rqt_dyros_gui.h>
#include <QDialog>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <iostream>

namespace rqt_dyros_gui
{
  class RqtDyrosPlugin : public rqt_gui_cpp::Plugin
  {
    Q_OBJECT

  public:
    RqtDyrosPlugin();
    virtual void initPlugin(qt_gui_cpp::PluginContext &context) override;
    virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings,
                              qt_gui_cpp::Settings &instance_settings) const override;
    virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                 const qt_gui_cpp::Settings &instance_settings) override;

  private:
    Ui::Dialog ui_;
    QDialog *widget_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr                     mode_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr    ee_pub_;

  private Q_SLOTS:
    void sendEECommand();
    void publishMode(int mode, const QString &label);
  };
}

#endif // RQT_DYROS_GUI_H
