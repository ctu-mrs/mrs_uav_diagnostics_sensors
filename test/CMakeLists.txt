find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)
find_package(mrs_uav_testing REQUIRED)

function(add_ros_isolated_launch_test path)
  set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
  add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
endfunction()

function(add_mrs_uav_diagnostics_sensors_test_target test_name)
  set(target_name "test_${test_name}")
  if(TARGET "${target_name}")
    message(FATAL_ERROR "Target for test name '${test_name}' already exists!")
  endif()

  add_executable(${target_name}
    test.cpp
  )

  target_link_libraries(${target_name}
    PRIVATE
      rclcpp::rclcpp
      mrs_lib::mrs_lib
      mrs_uav_testing::mrs_uav_testing
      nlohmann_json::nlohmann_json
      ${mrs_msgs_TARGETS}
      ${sensor_msgs_TARGETS}
  )

  install(
    TARGETS ${target_name}
    DESTINATION lib/${PROJECT_NAME}
  )

  add_ros_isolated_launch_test(test.py TIMEOUT 300)
endfunction()

add_subdirectory(sensor_status_smoke)
