#pragma once

#include <string>

#include <mrs_lib/param_loader.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace mrs_uav_diagnostics_sensors
{

/* resolvePluginConfig() //{ */

/**
 * @brief Loads a plugin's config into param_loader: an optional custom_config override, followed by the
 * plugin's own config file (plugin_config_path if set, otherwise this package's config/public/<config_key>.yaml
 * default), then sets the standard sensor_handlers/ prefix for subsequent loadParam() calls.
 */
inline void resolvePluginConfig(mrs_lib::ParamLoader &param_loader, const std::string &config_key, const std::string &plugin_config_path) {

  std::string custom_config_path;
  param_loader.loadParam("custom_config", custom_config_path, std::string(""));
  if (!custom_config_path.empty()) {
    param_loader.addYamlFile(custom_config_path);
  }

  const std::string resolved_config_path =
      plugin_config_path.empty() ? ament_index_cpp::get_package_share_directory("mrs_uav_diagnostics_sensors") + "/config/public/" + config_key + ".yaml"
                                 : plugin_config_path;
  param_loader.addYamlFile(resolved_config_path);
  param_loader.setPrefix("mrs_uav_managers/diagnostics_manager/sensor_handlers/");
}

//}

} // namespace mrs_uav_diagnostics_sensors
