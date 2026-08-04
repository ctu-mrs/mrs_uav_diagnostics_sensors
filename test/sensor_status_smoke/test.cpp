#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mrs_msgs/msg/sensor_info.hpp>
#include <mrs_msgs/msg/sensor_status.hpp>
#include <mrs_msgs/msg/system_health_info.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include <nlohmann/json.hpp>

#include <mrs_uav_testing/test_generic.h>

#include <thread>

using namespace std::chrono_literals;

// The multirotor simulator's hw_api never publishes 'hw_api/magnetic_field' (produces_magnetic_field is hardcoded to
// false in mrs_multirotor_simulator/src/hw_api_plugin.cpp) and nothing in the simulated stack publishes a camera_info
// topic at all. Both handlers would therefore sit at ready == false forever and their fill_details() would only ever
// produce empty/null content, which would make the checks below pass on nothing. The test node stands in for those two
// missing drivers so that the MagnetometerSensorHandler and the CameraSensorHandler are exercised on real data.
static const std::string CAMERA_INFO_TOPIC    = "servo_camera/camera_info";
static const std::string MAGNETIC_FIELD_TOPIC = "hw_api/magnetic_field";

// deliberately a bit above the rates expected by the handlers (60 Hz / 15 Hz): over-publishing keeps ready == true,
// under-publishing does not
static constexpr double CAMERA_INFO_RATE    = 70.0;
static constexpr double MAGNETIC_FIELD_RATE = 17.0;

static constexpr size_t EXPECTED_SENSOR_COUNT = 6;

class Tester : public mrs_uav_testing::TestGeneric {

public:
  Tester()
      : mrs_uav_testing::TestGeneric() {
  }

  bool test(void);

private:
  void publishCameraInfo(void);
  void publishMagneticField(void);

  rclcpp::Node::SharedPtr stand_in_node_;

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr    ph_camera_info_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr ph_magnetic_field_;

  rclcpp::TimerBase::SharedPtr timer_camera_info_;
  rclcpp::TimerBase::SharedPtr timer_magnetic_field_;

  std::string uav_name_;
};

/* publishCameraInfo() //{ */

void Tester::publishCameraInfo(void) {

  sensor_msgs::msg::CameraInfo msg;

  msg.header.stamp = stand_in_node_->get_clock()->now();
  // the handler transforms this frame into '<uav_name>/fcu', use a frame that the stack really publishes
  msg.header.frame_id = uav_name_ + "/fcu";

  msg.width  = 640;
  msg.height = 480;

  msg.distortion_model = "plumb_bob";
  msg.d                = {0.0, 0.0, 0.0, 0.0, 0.0};

  msg.k = {400.0, 0.0, 320.0, 0.0, 400.0, 240.0, 0.0, 0.0, 1.0};
  msg.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  msg.p = {400.0, 0.0, 320.0, 0.0, 0.0, 400.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0};

  ph_camera_info_->publish(msg);
}

//}

/* publishMagneticField() //{ */

void Tester::publishMagneticField(void) {

  sensor_msgs::msg::MagneticField msg;

  msg.header.stamp    = stand_in_node_->get_clock()->now();
  msg.header.frame_id = uav_name_ + "/fcu";

  // roughly the Earth's magnetic field in central Europe [T]
  msg.magnetic_field.x = 2.1e-5;
  msg.magnetic_field.y = 0.1e-5;
  msg.magnetic_field.z = 4.3e-5;

  msg.magnetic_field_covariance = {1e-12, 0.0, 0.0, 0.0, 1e-12, 0.0, 0.0, 0.0, 1e-12};

  ph_magnetic_field_->publish(msg);
}

//}

/* test() //{ */

bool Tester::test(void) {

  uav_name_ = "uav1";

  {
    auto [uhopt, message] = getUAVHandler(uav_name_);

    if (!uhopt) {
      RCLCPP_ERROR(node_->get_logger(), "Failed obtain handler for '%s': '%s'", uav_name_.c_str(), message.c_str());
      return false;
    }
  }

  // | ------------ stand in for the missing drivers ------------ |

  // the publishers live on their own node with their own executor thread, so that their timing is not affected by
  // everything else that is being spun by the test node
  stand_in_node_ = rclcpp::Node::make_shared("sensor_stand_in");

  auto qos = rclcpp::QoS(10).reliable().durability_volatile();

  ph_camera_info_    = stand_in_node_->create_publisher<sensor_msgs::msg::CameraInfo>("/" + uav_name_ + "/" + CAMERA_INFO_TOPIC, qos);
  ph_magnetic_field_ = stand_in_node_->create_publisher<sensor_msgs::msg::MagneticField>("/" + uav_name_ + "/" + MAGNETIC_FIELD_TOPIC, qos);

  timer_camera_info_ = stand_in_node_->create_wall_timer(std::chrono::duration<double>(1.0 / CAMERA_INFO_RATE), [this]() { this->publishCameraInfo(); });
  timer_magnetic_field_ =
      stand_in_node_->create_wall_timer(std::chrono::duration<double>(1.0 / MAGNETIC_FIELD_RATE), [this]() { this->publishMagneticField(); });

  rclcpp::executors::SingleThreadedExecutor stand_in_executor;
  stand_in_executor.add_node(stand_in_node_);

  std::thread stand_in_thread([&stand_in_executor]() { stand_in_executor.spin(); });

  // | ---------------------- subscribers ----------------------- |

  std::mutex mutex_msgs;

  std::shared_ptr<mrs_msgs::msg::SystemHealthInfo> health_msg;
  std::shared_ptr<mrs_msgs::msg::SensorInfo>       sensor_info_msg;

  // the manager publishes on '~/system_health_info_out', which mrs_uav_system.launch.py remaps to '~/system_health_info'
  auto sh_health = node_->create_subscription<mrs_msgs::msg::SystemHealthInfo>(
      "/" + uav_name_ + "/diagnostics_manager/system_health_info", rclcpp::SystemDefaultsQoS(), [&](const mrs_msgs::msg::SystemHealthInfo::SharedPtr msg) {
        std::scoped_lock lock(mutex_msgs);
        health_msg = msg;
      });

  auto sh_sensor_info = node_->create_subscription<mrs_msgs::msg::SensorInfo>("/" + uav_name_ + "/diagnostics_manager/sensor_info", rclcpp::SystemDefaultsQoS(),
                                                                              [&](const mrs_msgs::msg::SensorInfo::SharedPtr msg) {
                                                                                std::scoped_lock lock(mutex_msgs);
                                                                                sensor_info_msg = msg;
                                                                              });

  // | ---------------- wait for a healthy state ---------------- |

  // the stack needs to come up, then every handler has to get through its 5 s grace period and gather enough samples
  // to measure its rate
  const auto deadline = node_->get_clock()->now() + rclcpp::Duration(90s);

  std::shared_ptr<mrs_msgs::msg::SystemHealthInfo> last_health;
  std::shared_ptr<mrs_msgs::msg::SensorInfo>       last_sensor_info;

  bool all_healthy = false;

  while (rclcpp::ok() && node_->get_clock()->now() < deadline) {

    {
      std::scoped_lock lock(mutex_msgs);
      last_health      = health_msg;
      last_sensor_info = sensor_info_msg;
    }

    if (last_health && last_health->available_sensors.size() == EXPECTED_SENSOR_COUNT && last_sensor_info) {

      all_healthy = true;

      for (const auto &sensor : last_health->available_sensors) {
        if (!sensor.ready) {
          all_healthy = false;
          break;
        }
      }

      if (all_healthy) {
        break;
      }
    }

    sleep(0.2);
  }

  stand_in_executor.cancel();
  stand_in_thread.join();

  // | -------------------- report and check -------------------- |

  bool result = true;

  if (!last_health) {
    RCLCPP_ERROR(node_->get_logger(), "no message received on 'diagnostics_manager/system_health_info' within the deadline");
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "received SystemHealthInfo with %zu sensors", last_health->available_sensors.size());

  for (const auto &sensor : last_health->available_sensors) {

    RCLCPP_INFO(node_->get_logger(), "  sensor '%s': ready=%s, type=%u, level=%u, rate=%.2f Hz, topic='%s', message='%s'", sensor.name.c_str(),
                sensor.ready ? "true" : "false", sensor.type, sensor.level, sensor.rate, sensor.topic.c_str(), sensor.message.c_str());

    for (const auto &detail : sensor.details) {
      RCLCPP_INFO(node_->get_logger(), "    detail: '%s' = '%s'", detail.key.c_str(), detail.value.c_str());
    }

    if (!sensor.ready) {
      RCLCPP_ERROR(node_->get_logger(), "sensor '%s' is not ready: '%s'", sensor.name.c_str(), sensor.message.c_str());
      result = false;
    }
  }

  if (last_health->available_sensors.size() != EXPECTED_SENSOR_COUNT) {
    RCLCPP_ERROR(node_->get_logger(), "expected %zu sensor handlers, got %zu", EXPECTED_SENSOR_COUNT, last_health->available_sensors.size());
    result = false;
  }

  // | ----------- the camera's SensorInfo side-channel ---------- |

  if (!last_sensor_info) {
    RCLCPP_ERROR(node_->get_logger(), "no message received on 'diagnostics_manager/sensor_info' within the deadline");
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "received SensorInfo: type=%u, details='%s'", last_sensor_info->type, last_sensor_info->details.c_str());

  if (last_sensor_info->type != mrs_msgs::msg::SensorStatus::TYPE_CAMERA) {
    RCLCPP_ERROR(node_->get_logger(), "SensorInfo has type %u, expected TYPE_CAMERA (%u)", last_sensor_info->type, mrs_msgs::msg::SensorStatus::TYPE_CAMERA);
    result = false;
  }

  nlohmann::json details;

  try {
    details = nlohmann::json::parse(last_sensor_info->details);
  }
  catch (const nlohmann::json::exception &ex) {
    RCLCPP_ERROR(node_->get_logger(), "SensorInfo details do not parse as JSON: %s", ex.what());
    return false;
  }

  // don't check the exact numbers, just that the handler really filled the structure in from the camera_info it got
  if (!details.contains("camera_topic") || details["camera_topic"].get<std::string>() != CAMERA_INFO_TOPIC) {
    RCLCPP_ERROR(node_->get_logger(), "SensorInfo details do not carry the expected 'camera_topic'");
    result = false;
  }

  if (!details.contains("camera_info") || !details["camera_info"].is_object() || details["camera_info"].value("width", 0.0) <= 0.0 ||
      details["camera_info"].value("height", 0.0) <= 0.0 || !details["camera_info"].contains("fov_x_rad") || !details["camera_info"].contains("fov_y_rad")) {
    RCLCPP_ERROR(node_->get_logger(), "SensorInfo details do not carry a filled-in 'camera_info'");
    result = false;
  }

  if (!details.contains("camera_frame_tf") || !details["camera_frame_tf"].is_object() || !details["camera_frame_tf"].contains("translation") ||
      !details["camera_frame_tf"].contains("rotation_rpy")) {
    RCLCPP_ERROR(node_->get_logger(), "SensorInfo details do not carry a filled-in 'camera_frame_tf'");
    result = false;
  }

  if (!all_healthy) {
    RCLCPP_ERROR(node_->get_logger(), "the sensor handlers did not all report themselves ready within the deadline");
  }

  return result;
}

//}

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  bool test_result = true;

  Tester tester;

  test_result &= tester.test();

  tester.sleep(2.0);

  std::cout << "Test: reporting test results" << std::endl;

  tester.reportTestResult(test_result);

  tester.join();
}
