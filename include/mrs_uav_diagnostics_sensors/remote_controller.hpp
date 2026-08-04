#pragma once

#include <mrs_uav_managers/diagnostics_manager/diagnostics_sensor_handler.hpp>
#include <mrs_msgs/msg/hw_api_rc_rssi.hpp>

namespace mrs_uav_diagnostics_sensors
{

class RemoteControllerSensorHandler : public mrs_uav_managers::diagnostics_manager::DiagnosticsSensorHandler {
public:
  bool onInitialize(rclcpp::Node::SharedPtr &node, const std::string &config_key, const std::string &name_space, const std::string &plugin_config_path,
                    rclcpp::CallbackGroup::SharedPtr cbkgrp_subs = nullptr) override;

  std::vector<diagnostic_msgs::msg::KeyValue> fill_details() override;

private:
  mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiRcRssi> sh_rc_rssi_;
};

} // namespace mrs_uav_diagnostics_sensors
