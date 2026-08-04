#pragma once

#include <atomic>
#include <memory>

#include <mrs_uav_managers/diagnostics_manager/diagnostics_sensor_handler.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <mrs_msgs/msg/sensor_info.hpp>

#include <mrs_lib/transformer.h>

namespace mrs_uav_diagnostics_sensors
{

class CameraSensorHandler : public mrs_uav_managers::diagnostics_manager::DiagnosticsSensorHandler {
public:
  CameraSensorHandler() = default;

  bool onInitialize(rclcpp::Node::SharedPtr &node, const std::string &config_key, const std::string &name_space, const std::string &plugin_config_path,
                    rclcpp::CallbackGroup::SharedPtr cbkgrp_subs = nullptr) override;

  std::vector<diagnostic_msgs::msg::KeyValue> fill_details() override;


private:
  std::string _fcu_frame_;

  mrs_lib::SubscriberHandler<sensor_msgs::msg::CameraInfo>     sh_camera_info_;
  mrs_lib::SubscriberHandler<std_msgs::msg::Float32MultiArray> sh_camera_gimbal_orientation_;
  std::atomic_bool                                             use_camera_gimbal_orientation_ = false;

  mrs_lib::PublisherHandler<mrs_msgs::msg::SensorInfo> ph_camera_details_;

  std::unique_ptr<mrs_lib::Transformer> transformer_;
};

} // namespace mrs_uav_diagnostics_sensors
