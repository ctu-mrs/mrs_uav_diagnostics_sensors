#pragma once

#include <mrs_uav_managers/diagnostics_manager/diagnostics_sensor_handler.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

namespace mrs_uav_diagnostics_sensors
{
namespace magnetometer_handler
{

class MagnetometerSensorHandler : public mrs_uav_managers::DiagnosticsSensorHandler {
public:
  bool onInitialize(rclcpp::Node::SharedPtr &node, const std::string &config_key, const std::string &name_space, const std::string &plugin_config_path,
                    rclcpp::CallbackGroup::SharedPtr cbkgrp_subs = nullptr) override;

  std::vector<diagnostic_msgs::msg::KeyValue> fill_details() override;

private:
  mrs_lib::SubscriberHandler<sensor_msgs::msg::MagneticField> sh_magnetic_field_;
};

} // namespace magnetometer_handler
} // namespace mrs_uav_diagnostics_sensors
