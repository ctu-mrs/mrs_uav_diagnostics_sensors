# MRS UAV Diagnostics Sensors

Concrete `DiagnosticsSensorHandler` plugin implementations for `mrs_uav_managers`'s
`DiagnosticsManager`: GNSS, camera, generic (message-agnostic), magnetometer, and
remote controller.

This package ships pluginlib plugins only — it has no standalone node and is never run
directly. Plugin instances live entirely in `mrs_uav_managers`'s own
`config/public/diagnostics_manager/sensor_handlers.yaml`: which instances are
enabled (`active_sensor_handlers`), and each instance's pluginlib class address, tunables
(`type`/`topic`/`expected_rate`/...), and any plugin-specific fields (e.g. `message_type`,
`status_topic`).

Plugin-specific fields not shown above are resolved by
`resolvePluginConfig()` (`include/mrs_uav_diagnostics_sensors/utils/plugin_config.hpp`), which
also supports an explicit `plugin_config` override path per instance for cases that want an
external file instead.
