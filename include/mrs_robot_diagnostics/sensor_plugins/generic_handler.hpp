#pragma once

#include <mrs_lib/param_loader.h>
#include <mrs_uav_managers/diagnostics_manager/diagnostics_sensor_handler.hpp>
#include <rclcpp/generic_subscription.hpp>

namespace mrs_robot_diagnostics
{
namespace generic_handler
{

class GenericSensorHandler : public mrs_uav_managers::DiagnosticsSensorHandler {
public:
  bool onInitialize(rclcpp::Node::SharedPtr &node, const std::string &config_key, const std::string &name_space, const std::string &plugin_config_path,
                    rclcpp::CallbackGroup::SharedPtr cbkgrp_subs = nullptr) override final;

private:
  // Generic subscription (type-erased)
  std::shared_ptr<rclcpp::GenericSubscription> generic_sub_;
  // Cant use createSubscriber<T> because it returns a SubscriberHandler<T>, but we need a GenericSubscription for type erasure. So we implement the
  // subscription and callback manually here, but reuse the same timestamp tracking and rate calculation logic from the base class.
  void messageCallback(const std::shared_ptr<const rclcpp::SerializedMessage> &msg);
};

} // namespace generic_handler
} // namespace mrs_robot_diagnostics
