#pragma once

#include <mrs_lib/param_loader.h>
#include <mrs_uav_managers/diagnostics_manager/diagnostics_sensor_handler.hpp>
#include <rclcpp/generic_subscription.hpp>

namespace mrs_uav_diagnostics_sensors
{

class GenericSensorHandler : public mrs_uav_managers::diagnostics_manager::DiagnosticsSensorHandler {
public:
  // Loads the required message_type config field and creates a type-erased generic subscription for it on the main topic --
  // status is rate/staleness only (fill_details() is not overridden), since the message type isn't known at compile time.
  bool onInitialize(rclcpp::Node::SharedPtr &node, const std::string &config_key, const std::string &name_space, const std::string &plugin_config_path,
                    rclcpp::CallbackGroup::SharedPtr cbkgrp_subs = nullptr) override final;

private:
  // Generic subscription (type-erased)
  std::shared_ptr<rclcpp::GenericSubscription> generic_sub_;
  // Cant use createSubscriber<T> because it returns a SubscriberHandler<T>, but we need a GenericSubscription for type erasure. So we implement the
  // subscription and callback manually here, but reuse the same timestamp tracking and rate calculation logic from the base class.
  void messageCallback(const std::shared_ptr<const rclcpp::SerializedMessage> &msg);
};

} // namespace mrs_uav_diagnostics_sensors
