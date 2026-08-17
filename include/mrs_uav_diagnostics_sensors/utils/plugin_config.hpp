#pragma once

#include <filesystem>
#include <string>

#include <mrs_lib/param_loader.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace mrs_uav_diagnostics_sensors
{

/* resolvePluginConfig() //{ */

/**
 * @brief Loads a plugin's config into param_loader: an optional custom_config override, followed by the
 * plugin's own config file (plugin_config_path if set, otherwise this package's config/public/<config_key>.yaml
 * default, if it exists), then sets the standard sensor_handlers/ prefix for subsequent loadParam() calls.
 *
 * A user-provided plugin_config_path that doesn't exist is a real config error and is loaded (and thus
 * logged loudly by ParamLoader) unconditionally. The implicit config_key-based default, on the other hand,
 * only exists for the handful of instances this package ships (e.g. PX4.yaml) -- for any other instance
 * name (e.g. a deployment's own custom sensor) it's expected to be absent, so it's skipped silently instead
 * of logging a "file does not exist" error about a package the caller doesn't own. Any field this plugin
 * still can't find afterwards (e.g. message_type) fails on its own loadParam() call with a clear message.
 */
inline void resolvePluginConfig(mrs_lib::ParamLoader &param_loader, const std::string &config_key, const std::string &plugin_config_path) {

  std::string custom_config_path;
  param_loader.loadParam("custom_config", custom_config_path, std::string(""));
  if (!custom_config_path.empty()) {
    param_loader.addYamlFile(custom_config_path);
  }

  if (!plugin_config_path.empty()) {
    param_loader.addYamlFile(plugin_config_path);
  } else {
    const std::string default_config_path = ament_index_cpp::get_package_share_directory("mrs_uav_diagnostics_sensors") + "/config/public/" + config_key + ".yaml";
    if (std::filesystem::exists(default_config_path)) {
      param_loader.addYamlFile(default_config_path);
    }
  }
  param_loader.setPrefix("mrs_uav_managers/diagnostics_manager/sensor_handlers/");
}

//}

} // namespace mrs_uav_diagnostics_sensors
