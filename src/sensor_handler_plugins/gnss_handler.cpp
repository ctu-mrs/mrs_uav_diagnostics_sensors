#include <mrs_uav_diagnostics_sensors/sensor_plugins/gnss_handler.hpp>

#include <mrs_uav_diagnostics_sensors/utils/detail_builder.hpp>
#include <mrs_uav_diagnostics_sensors/utils/plugin_config.hpp>

namespace mrs_uav_diagnostics_sensors::gnss_handler
{

/* onInitialize() //{ */

bool GNSSSensorHandler::onInitialize(rclcpp::Node::SharedPtr &node, const std::string &config_key, [[maybe_unused]] const std::string &name_space,
                                     const std::string &plugin_config_path, [[maybe_unused]] rclcpp::CallbackGroup::SharedPtr cbkgrp_subs) {

  mrs_lib::ParamLoader param_loader(node, "GNSSSensorHandler");

  resolvePluginConfig(param_loader, config_key, plugin_config_path);

  // Read GNSSSensorHandler-specific params
  std::string status_topic;
  param_loader.loadParam(config_key + "/status_topic", status_topic);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node->get_logger(), "[GNSSSensorHandler] Failed to load config for '%s', not initializing", config_key.c_str());
    error_publisher_->addOneshotError("Failed to load config for " + name_);
    return false;
  }

  RCLCPP_INFO(node->get_logger(), "[GNSSSensorHandler] Initializing '%s', topic: '%s'", name_.c_str(), topic_.c_str());
  RCLCPP_INFO(node->get_logger(), "[GNSSSensorHandler] Initializing '%s', topic: '%s'", name_.c_str(), status_topic.c_str());

  // Create subscriber handlers for GNSS data and status
  sh_gnns_        = create_main_subscriber<sensor_msgs::msg::NavSatFix>(node, topic_, cbkgrp_subs);
  sh_gnss_status_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::GpsInfo>(shopts_, status_topic);
  return true;
}

//}

/* fill_details() //{ */

std::vector<diagnostic_msgs::msg::KeyValue> GNSSSensorHandler::fill_details() {

  std::vector<diagnostic_msgs::msg::KeyValue> details;

  auto gnss_msg        = sh_gnns_.getMsg();
  auto gnss_status_msg = sh_gnss_status_.getMsg();

  std::optional<double> uncertainty, quality;
  if (gnss_msg) {
    const Eigen::Matrix3d cov = cov2eigen(gnss_msg->position_covariance);
    uncertainty               = std::pow(cov.determinant(), 1.0 / 6.0);
    quality                   = (gnss_msg->position_covariance[0] + gnss_msg->position_covariance[4] + gnss_msg->position_covariance[8]) / 3;
  }
  details.push_back(make_detail("uncertainty", uncertainty));
  details.push_back(make_detail("quality", quality));

  if (!gnss_status_msg || !isTopicFresh(shopts_.node->get_clock()->now(), sh_gnss_status_.lastMsgTime())) {
    details.push_back(make_detail("fix_type", "nan"));
    details.push_back(make_detail("num_satellites", "nan"));
    details.push_back(make_detail("position_accuracy", "nan"));
  } else {
    const double position_accuracy = (gnss_status_msg->h_acc + gnss_status_msg->v_acc) / 2.0;
    // GpsInfo.msg: satellites_visible is set to 255 when unknown.
    const std::string num_satellites_str = (gnss_status_msg->satellites_visible == 255) ? "nan" : std::to_string(gnss_status_msg->satellites_visible);
    details.push_back(make_detail("fix_type", std::to_string(gnss_status_msg->fix_type)));
    details.push_back(make_detail("num_satellites", num_satellites_str));
    details.push_back(make_detail("position_accuracy", std::to_string(position_accuracy)));
  }

  return details;
}

//}

} // namespace mrs_uav_diagnostics_sensors::gnss_handler

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_uav_diagnostics_sensors::gnss_handler::GNSSSensorHandler, mrs_uav_managers::diagnostics_manager::DiagnosticsSensorHandler)
