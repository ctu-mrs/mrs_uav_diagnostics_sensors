#pragma once

#include <mrs_uav_managers/diagnostics_manager/diagnostics_sensor_handler.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

namespace mrs_uav_diagnostics_sensors
{

class MagnetometerSensorHandler : public mrs_uav_managers::diagnostics_manager::DiagnosticsSensorHandler {
public:
  // No plugin-specific config fields; subscribes to MagneticField on the main topic.
  bool onInitialize(rclcpp::Node::SharedPtr &node, const std::string &config_key, const std::string &name_space, const std::string &plugin_config_path,
                    rclcpp::CallbackGroup::SharedPtr cbkgrp_subs = nullptr) override;

  // Reports uncertainty (from the field covariance), strength (field norm, Tesla), and norm_gauss (the same norm
  // converted to Gauss for the TUI) computed from the latest MagneticField message.
  std::vector<diagnostic_msgs::msg::KeyValue> fill_details() override;

private:
  mrs_lib::SubscriberHandler<sensor_msgs::msg::MagneticField> sh_magnetic_field_;
};

} // namespace mrs_uav_diagnostics_sensors
