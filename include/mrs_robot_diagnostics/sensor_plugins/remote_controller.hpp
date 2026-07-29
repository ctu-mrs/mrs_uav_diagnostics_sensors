#pragma once

#include <mrs_uav_managers/diagnostics_manager/diagnostics_sensor_handler.hpp>
#include <mrs_msgs/msg/hw_api_rc_rssi.hpp>

namespace mrs_robot_diagnostics
{
namespace rc_handler
{

class RCSensorHandler : public mrs_uav_managers::DiagnosticsSensorHandler {
public:
  bool onInitialize(rclcpp::Node::SharedPtr &node, const std::string &config_key, const std::string &name_space,
                    rclcpp::CallbackGroup::SharedPtr cbkgrp_subs = nullptr) override;

  std::vector<diagnostic_msgs::msg::KeyValue> fill_details() override;

private:
  mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiRcRssi> sh_rc_rssi_;
};

} // namespace rc_handler
} // namespace mrs_robot_diagnostics
