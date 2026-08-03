cmake_minimum_required(VERSION 3.28)
project(mrs_uav_diagnostics_sensors)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[mrs_uav_diagnostics_sensors]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

set(DEPENDENCIES
  rclcpp
  mrs_msgs
  mrs_lib
  mrs_uav_managers
  tf2_ros
  pluginlib
  diagnostic_msgs
  nlohmann_json
  ament_index_cpp
)

set(LIBRARIES
  MrsUavDiagnosticsSensors_SensorPlugins
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

## | -------------- MrsUavDiagnosticsSensors_SensorPlugins --------------- |

add_library(MrsUavDiagnosticsSensors_SensorPlugins SHARED
  src/sensor_handler_plugins/camera_handler.cpp
  src/sensor_handler_plugins/generic_handler.cpp
  src/sensor_handler_plugins/gnss_handler.cpp
  src/sensor_handler_plugins/magnetometer_handler.cpp
  src/sensor_handler_plugins/remote_controller.cpp
)

target_sources(MrsUavDiagnosticsSensors_SensorPlugins
  PUBLIC FILE_SET HEADERS BASE_DIRS "include"
  FILES
    "include/mrs_uav_diagnostics_sensors/sensor_plugins/camera_handler.hpp"
    "include/mrs_uav_diagnostics_sensors/sensor_plugins/generic_handler.hpp"
    "include/mrs_uav_diagnostics_sensors/sensor_plugins/gnss_handler.hpp"
    "include/mrs_uav_diagnostics_sensors/sensor_plugins/magnetometer_handler.hpp"
    "include/mrs_uav_diagnostics_sensors/sensor_plugins/remote_controller.hpp"
)

target_link_libraries(MrsUavDiagnosticsSensors_SensorPlugins
  PUBLIC
    mrs_uav_managers::mrs_uav_managers
  PRIVATE
    nlohmann_json::nlohmann_json
    ament_index_cpp::ament_index_cpp
)

target_compile_definitions(MrsUavDiagnosticsSensors_SensorPlugins PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

## --------------------------------------------------------------
## |                       Export plugins                       |
## --------------------------------------------------------------

pluginlib_export_plugin_description_file(mrs_uav_managers plugins.xml)

## --------------------------------------------------------------
## |                           Testing                          |
## --------------------------------------------------------------

if(ENABLE_TESTS)

  find_package(ament_cmake_clang_format REQUIRED)
  ament_clang_format(CONFIG_FILE "${PROJECT_SOURCE_DIR}/.clang-format")

  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()

endif()

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(
  TARGETS
    ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  FILE_SET HEADERS DESTINATION "include/${PROJECT_NAME}"
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)

ament_export_libraries(
  ${LIBRARIES}
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
