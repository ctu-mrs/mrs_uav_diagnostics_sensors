#pragma once

#include <optional>
#include <string>

#include <diagnostic_msgs/msg/key_value.hpp>

namespace mrs_uav_diagnostics_sensors
{

/* make_detail() //{ */

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

/**
 * @brief Build a KeyValue detail entry from an optional double, using "nan" when the value isn't available.
 * Only meant for fields that are genuinely floating-point -- integer fields formatted without decimals
 * (e.g. fix_type, num_satellites, rssi) should keep their own std::to_string()-based handling.
 */
inline diagnostic_msgs::msg::KeyValue make_detail(const std::string &key, const std::optional<double> &value) {
  return make_detail(key, value ? std::to_string(*value) : "nan");
}

//}

} // namespace mrs_uav_diagnostics_sensors
