#pragma once

#include <mrs_uav_managers/diagnostics_manager/diagnostics_sensor_handler.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <mrs_msgs/msg/gps_info.hpp>

namespace mrs_uav_diagnostics_sensors
{

class GNSSSensorHandler : public mrs_uav_managers::diagnostics_manager::DiagnosticsSensorHandler {
public:
  // Loads the required status_topic config field, subscribes to NavSatFix on the main topic and GpsInfo on status_topic.
  bool onInitialize(rclcpp::Node::SharedPtr &node, const std::string &config_key, const std::string &name_space, const std::string &plugin_config_path,
                    rclcpp::CallbackGroup::SharedPtr cbkgrp_subs = nullptr) override;

  // Reports position uncertainty/quality from NavSatFix's covariance, plus fix_type/num_satellites/position_accuracy
  // from GpsInfo (or "nan" for the latter three if GpsInfo hasn't arrived or has gone stale).
  std::vector<diagnostic_msgs::msg::KeyValue> fill_details() override;

private:
  mrs_lib::SubscriberHandler<sensor_msgs::msg::NavSatFix> sh_gnns_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::GpsInfo>      sh_gnss_status_;
};

} // namespace mrs_uav_diagnostics_sensors
