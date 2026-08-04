#include <mrs_uav_diagnostics_sensors/sensor_plugins/magnetometer_handler.hpp>
#include <mrs_uav_diagnostics_sensors/utils/detail_builder.hpp>

namespace mrs_uav_diagnostics_sensors::magnetometer_handler
{

/* onInitialize() //{ */

bool MagnetometerSensorHandler::onInitialize(rclcpp::Node::SharedPtr &node, [[maybe_unused]] const std::string &config_key,
                                             [[maybe_unused]] const std::string &name_space, [[maybe_unused]] const std::string &plugin_config_path,
                                             [[maybe_unused]] rclcpp::CallbackGroup::SharedPtr cbkgrp_subs) {

  RCLCPP_INFO(node->get_logger(), "[%s]: initializing, topic: '%s'", name_.c_str(), topic_.c_str());

  sh_magnetic_field_ = create_main_subscriber<sensor_msgs::msg::MagneticField>(node, topic_, cbkgrp_subs);
  return true;
}

//}

/* fill_details() //{ */

std::vector<diagnostic_msgs::msg::KeyValue> MagnetometerSensorHandler::fill_details() {

  std::vector<diagnostic_msgs::msg::KeyValue> details;

  auto magnetic_field_msg = sh_magnetic_field_.getMsg();

  std::optional<double> uncertainty, strength, norm_gauss;
  if (magnetic_field_msg) {
    const Eigen::Matrix3d cov = cov2eigen(magnetic_field_msg->magnetic_field_covariance);
    const Eigen::Vector3d mag(magnetic_field_msg->magnetic_field.x, magnetic_field_msg->magnetic_field.y, magnetic_field_msg->magnetic_field.z);
    const double          norm_tesla = mag.norm();

    uncertainty = std::pow(cov.determinant(), 1.0 / 6.0);
    strength    = norm_tesla;
    // sensor_msgs/MagneticField publishes Tesla; the TUI consumes Gauss (1 T = 1e4 G).
    norm_gauss = norm_tesla * 1.0e4;
  }
  details.push_back(make_detail("uncertainty", uncertainty));
  details.push_back(make_detail("strength", strength));
  details.push_back(make_detail("norm_gauss", norm_gauss));
  details.push_back(make_detail("norm_hz", std::to_string(getMeasuredRate())));

  return details;
}

//}

} // namespace mrs_uav_diagnostics_sensors::magnetometer_handler

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_uav_diagnostics_sensors::magnetometer_handler::MagnetometerSensorHandler,
                       mrs_uav_managers::diagnostics_manager::DiagnosticsSensorHandler)
