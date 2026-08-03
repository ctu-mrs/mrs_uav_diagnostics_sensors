# MRS UAV Diagnostics Sensors

Concrete `DiagnosticsSensorHandler` plugin implementations for `mrs_uav_managers`'s
`DiagnosticsManager`: GNSS, camera, generic (message-agnostic), magnetometer, and
remote controller.

This package ships pluginlib plugins only — it has no standalone node and is never run
directly. Plugin instances live in `mrs_uav_managers`'s own config: which instances exist
and their tunables (`type`/`topic`/`expected_rate`/...) are in
`config/public/diagnostics_manager/diagnostics_sensor_handlers.yaml`, which of those are
actually enabled is `active_sensor_handlers` in
`config/public/diagnostics_manager/diagnostics_manager.yaml`, and each instance's
pluginlib class address is in
`config/private/diagnostics_manager/diagnostics_sensor_handlers.yaml`. `DiagnosticsManager`
loads them via pluginlib at runtime.

Each plugin falls back to its own default config under `config/sensor_plugins/<key>.yaml`
(keyed by the plugin instance's config key, e.g. `GNSS.yaml`) whenever the central config
doesn't set a `plugin_config` override for that instance.
