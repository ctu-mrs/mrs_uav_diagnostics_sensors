#include <mrs_uav_diagnostics_sensors/sensor_plugins/camera_handler.hpp>

#include <cmath>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <nlohmann/json.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace mrs_uav_diagnostics_sensors::camera_handler
{
/* onInitialize() //{ */

bool CameraSensorHandler::onInitialize(rclcpp::Node::SharedPtr &node, const std::string &config_key, const std::string &name_space,
                                       const std::string &plugin_config_path, rclcpp::CallbackGroup::SharedPtr cbkgrp_subs) {

  mrs_lib::ParamLoader param_loader(node, "CameraSensorHandler");

  std::string custom_config_path;
  param_loader.loadParam("custom_config", custom_config_path, std::string(""));
  if (!custom_config_path.empty()) {
    param_loader.addYamlFile(custom_config_path);
  }

  const std::string resolved_config_path = plugin_config_path.empty() ? ament_index_cpp::get_package_share_directory("mrs_uav_diagnostics_sensors") +
                                                                            "/config/sensor_plugins/" + config_key + ".yaml"
                                                                      : plugin_config_path;
  param_loader.addYamlFile(resolved_config_path);
  param_loader.setPrefix("mrs_uav_managers/diagnostics_manager/sensor_handlers/");

  // Read CameraSensorHandler-specific params
  std::string gimbal_orientation_topic;
  param_loader.loadParam(config_key + "/gimbal_orientation_topic", gimbal_orientation_topic, std::string(""));

  std::string sensor_info_publisher_topic;
  param_loader.loadParam(config_key + "/sensor_info_publisher_topic", sensor_info_publisher_topic, std::string("~/sensor_info"));

  param_loader.loadParam(config_key + "/fcu_frame", _fcu_frame_, std::string(name_space + "/fcu"));

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node->get_logger(), "[CameraSensorHandler] Failed to load config for '%s', not initializing", name_.c_str());
    return false;
  }

  // Initialize tf2 components
  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node);

  // Create subscriber
  RCLCPP_INFO(node->get_logger(), "[CameraSensorHandler] Initializing '%s', topic: '%s'", name_.c_str(), topic_.c_str());
  sh_camera_info_ = create_main_subscriber<sensor_msgs::msg::CameraInfo>(node, topic_, cbkgrp_subs);

  if (!gimbal_orientation_topic.empty()) {
    RCLCPP_INFO(node->get_logger(), "[CameraSensorHandler] Initializing '%s', topic: '%s'", name_.c_str(), gimbal_orientation_topic.c_str());
    sh_camera_gimbal_orientation_  = mrs_lib::SubscriberHandler<std_msgs::msg::Float32MultiArray>(shopts_, gimbal_orientation_topic);
    use_camera_gimbal_orientation_ = true;
  }

  // create publisher
  RCLCPP_INFO(node->get_logger(), "[CameraSensorHandler] Initializing '%s', topic: '%s'", name_.c_str(), sensor_info_publisher_topic.c_str());
  ph_camera_details_ = mrs_lib::PublisherHandler<mrs_msgs::msg::SensorInfo>(node, sensor_info_publisher_topic);

  is_initialized_ = true;
  return true;
}

//}

/* fill_details() //{ */

std::vector<diagnostic_msgs::msg::KeyValue> CameraSensorHandler::fill_details() {

  geometry_msgs::msg::TransformStamped transform;
  nlohmann::json                       camera_tf_json;
  nlohmann::json                       camera_info_json;
  if (sh_camera_info_.hasMsg()) {

    auto         msg    = sh_camera_info_.getMsg();
    const double height = msg->height;
    const double width  = msg->width;
    const double fx     = msg->k[0];
    const double fy     = msg->k[4];

    camera_info_json = {
        {"height", height},
        {"width", width},
    };

    if (fx > 0.0 && fy > 0.0) {
      camera_info_json["fov_x_rad"] = 2 * atan(width / (2 * fx));
      camera_info_json["fov_y_rad"] = 2 * atan(height / (2 * fy));
    }

    try {
      transform = tf_buffer_->lookupTransform(_fcu_frame_, sh_camera_info_.getMsg()->header.frame_id, tf2::TimePointZero);
      double x  = transform.transform.translation.x;
      double y  = transform.transform.translation.y;
      double z  = transform.transform.translation.z;

      double qx = transform.transform.rotation.x;
      double qy = transform.transform.rotation.y;
      double qz = transform.transform.rotation.z;
      double qw = transform.transform.rotation.w;

      tf2::Quaternion q(qx, qy, qz, qw);
      double          roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      camera_tf_json = {
          {"translation", {{"x", x}, {"y", y}, {"z", z}}},
          {"rotation_rpy", {{"roll", roll}, {"pitch", pitch}, {"yaw", yaw}}},
      };
    }
    catch (tf2::TransformException &ex) {
      RCLCPP_WARN(rclcpp::get_logger("CameraSensorHandler"), "%s", ex.what());
    }
  }

  nlohmann::json camera_orientation_json;
  if (use_camera_gimbal_orientation_ && sh_camera_gimbal_orientation_.hasMsg() &&
      isTopicFresh(shopts_.node->get_clock()->now(), sh_camera_gimbal_orientation_.lastMsgTime())) {
    const auto   orientation_msg = sh_camera_gimbal_orientation_.getMsg();
    const double roll            = (orientation_msg->data.size() > 0) ? orientation_msg->data[0] : 0.0;
    const double pitch           = (orientation_msg->data.size() > 1) ? orientation_msg->data[1] : 0.0;
    const double yaw             = (orientation_msg->data.size() > 2) ? orientation_msg->data[2] : 0.0;

    camera_orientation_json = {
        {"orientation_rpy",
         {
             {"roll", roll},
             {"pitch", pitch},
             {"yaw", yaw},
         }},
    };
  }

  nlohmann::json json_msg = {
      {"camera_topic", topic_},
      {"camera_frame_tf", camera_tf_json},
      {"camera_info", camera_info_json},
      {"camera_orientation", camera_orientation_json},
  };

  std::string json_str = json_msg.dump();

  mrs_msgs::msg::SensorInfo sensor_info_msg;
  sensor_info_msg.type    = mrs_msgs::msg::SensorStatus::TYPE_CAMERA;
  sensor_info_msg.details = json_str;
  ph_camera_details_.publish(sensor_info_msg);

  return {};
}

//}

} // namespace mrs_uav_diagnostics_sensors::camera_handler

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_uav_diagnostics_sensors::camera_handler::CameraSensorHandler, mrs_uav_managers::diagnostics_manager::DiagnosticsSensorHandler)
