#include <mrs_uav_diagnostics_sensors/sensor_plugins/remote_controller.hpp>
#include <mrs_uav_diagnostics_sensors/utils/detail_builder.hpp>

namespace mrs_uav_diagnostics_sensors::rc_handler
{

/* onInitialize() //{ */

bool RCSensorHandler::onInitialize(rclcpp::Node::SharedPtr &node, [[maybe_unused]] const std::string &config_key,
                                   [[maybe_unused]] const std::string &name_space, [[maybe_unused]] const std::string &plugin_config_path,
                                   [[maybe_unused]] rclcpp::CallbackGroup::SharedPtr cbkgrp_subs) {

  RCLCPP_INFO(node->get_logger(), "[%s]: initializing, topic: '%s'", name_.c_str(), topic_.c_str());

  sh_rc_rssi_ = create_main_subscriber<mrs_msgs::msg::HwApiRcRssi>(node, topic_, cbkgrp_subs);
  return true;
}

//}

/* fill_details() //{ */

std::vector<diagnostic_msgs::msg::KeyValue> RCSensorHandler::fill_details() {

  std::vector<diagnostic_msgs::msg::KeyValue> details;

  auto rc_rssi_msg = sh_rc_rssi_.getMsg();

  if (!rc_rssi_msg) {
    details.push_back(make_detail("rssi", "nan"));
  } else {
    details.push_back(make_detail("rssi", std::to_string(rc_rssi_msg->rssi)));
  }

  return details;
}

//}

} // namespace mrs_uav_diagnostics_sensors::rc_handler

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_uav_diagnostics_sensors::rc_handler::RCSensorHandler, mrs_uav_managers::diagnostics_manager::DiagnosticsSensorHandler)
