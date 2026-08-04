#include <mrs_uav_diagnostics_sensors/sensor_plugins/generic_handler.hpp>

#include <mrs_uav_diagnostics_sensors/utils/plugin_config.hpp>

namespace mrs_uav_diagnostics_sensors
{

/* onInitialize() //{ */

bool GenericSensorHandler::onInitialize(rclcpp::Node::SharedPtr &node, const std::string &config_key, [[maybe_unused]] const std::string &name_space,
                                        const std::string &plugin_config_path, rclcpp::CallbackGroup::SharedPtr cbkgrp_subs) {

  mrs_lib::ParamLoader param_loader(node, "GenericSensorHandler");

  resolvePluginConfig(param_loader, config_key, plugin_config_path);

  // Read GenericSensorHandler-specific params
  std::string message_type;
  param_loader.loadParam(config_key + "/message_type", message_type);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node->get_logger(), "[%s]: failed to load config, not initializing", config_key.c_str());
    error_publisher_->addOneshotError("Failed to load config for " + name_);
    return false;
  }

  // Create the generic subscription
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cbkgrp_subs;

  generic_sub_ = node->create_generic_subscription(
      topic_, message_type, qos_profile_, [this](std::shared_ptr<const rclcpp::SerializedMessage> msg) { this->messageCallback(msg); }, sub_options);

  RCLCPP_INFO(node->get_logger(), "[%s]: initialized: topic='%s', msg_type='%s', expected_rate=%.1f Hz, tolerance=%.0f%%", name_.c_str(), topic_.c_str(),
              message_type.c_str(), expected_rate_, rate_tolerance_ * 100.0);

  return true;
}

//}

/* messageCallback() //{ */

void GenericSensorHandler::messageCallback([[maybe_unused]] const std::shared_ptr<const rclcpp::SerializedMessage> &msg) {
  const rclcpp::Time now = rclcpp::Clock(RCL_STEADY_TIME).now();
  rate_tracker_.record(now);
  {
    std::scoped_lock lock(mutex_state_);
    state_.msg_count++;
    state_.last_msg_wall_time = now;
  }
}

//}

} // namespace mrs_uav_diagnostics_sensors

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_uav_diagnostics_sensors::GenericSensorHandler, mrs_uav_managers::diagnostics_manager::DiagnosticsSensorHandler)
