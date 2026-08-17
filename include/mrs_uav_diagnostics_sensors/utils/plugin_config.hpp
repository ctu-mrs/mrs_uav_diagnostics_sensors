#pragma once

#include <filesystem>
#include <string>

#include <mrs_lib/param_loader.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace mrs_uav_diagnostics_sensors
{

/* resolvePluginConfig() //{ */

/**
 * @brief Loads a plugin's config into param_loader, highest priority first: custom_config, an explicit
 * plugin_config_path, DiagnosticsManager's own public_sensor_handlers file (so plugin-specific fields like
 * message_type can just live next to type/topic/... there), then this package's config/public/<config_key>.yaml
 * default, if it exists. Sets the standard sensor_handlers/ prefix afterwards.
 *
 * A missing plugin_config_path fails loudly (real config error). A missing implicit default is skipped
 * silently, since it's only expected to exist for the few instances this package ships one for.
 */
inline void resolvePluginConfig(mrs_lib::ParamLoader &param_loader, const std::string &config_key, const std::string &plugin_config_path) {

  std::string custom_config_path;
  param_loader.loadParam("custom_config", custom_config_path, std::string(""));
  if (!custom_config_path.empty()) {
    param_loader.addYamlFile(custom_config_path);
  }

  if (!plugin_config_path.empty()) {
    param_loader.addYamlFile(plugin_config_path);
  }

  param_loader.addYamlFileFromParam("public_sensor_handlers");

  if (plugin_config_path.empty()) {
    const std::string default_config_path = ament_index_cpp::get_package_share_directory("mrs_uav_diagnostics_sensors") + "/config/public/" + config_key + ".yaml";
    if (std::filesystem::exists(default_config_path)) {
      param_loader.addYamlFile(default_config_path);
    }
  }
  param_loader.setPrefix("mrs_uav_managers/diagnostics_manager/sensor_handlers/");
}

//}

} // namespace mrs_uav_diagnostics_sensors
