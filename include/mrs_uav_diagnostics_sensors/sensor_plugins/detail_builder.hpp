#pragma once

#include <string>

#include <diagnostic_msgs/msg/key_value.hpp>

namespace mrs_uav_diagnostics_sensors
{

/**
 * @brief Build a single diagnostic_msgs::msg::KeyValue detail entry.
 * Used by fill_details() overrides to cut the repeated key/value/push_back boilerplate.
 */
inline diagnostic_msgs::msg::KeyValue make_detail(const std::string &key, const std::string &value) {
  diagnostic_msgs::msg::KeyValue info;
  info.key   = key;
  info.value = value;
  return info;
}

} // namespace mrs_uav_diagnostics_sensors
