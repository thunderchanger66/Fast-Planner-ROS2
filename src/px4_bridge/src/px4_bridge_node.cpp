#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{
using Vector3 = std::array<double, 3>;
using Matrix3 = std::array<double, 9>;
using PositionCommand = quadrotor_msgs::msg::PositionCommand;

bool finite(const Vector3 & vector)
{
  return std::all_of(vector.begin(), vector.end(), [](double value) {
    return std::isfinite(value);
  });
}

Vector3 rotate(const Matrix3 & matrix, const Vector3 & vector)
{
  return {
    matrix[0] * vector[0] + matrix[1] * vector[1] + matrix[2] * vector[2],
    matrix[3] * vector[0] + matrix[4] * vector[1] + matrix[5] * vector[2],
    matrix[6] * vector[0] + matrix[7] * vector[1] + matrix[8] * vector[2]};
}

struct Setpoint
{
  Vector3 position{};
  Vector3 velocity{};
  Vector3 acceleration{};
  double yaw{0.0};
  double yaw_rate{0.0};
};

class PlannerPx4Bridge final : public rclcpp::Node
{
public:
  PlannerPx4Bridge()
  : Node("px4_bridge")
  {
    declare_parameter("planner_command_topic", "/planner/position_cmd");
    declare_parameter("planner_frame", "lidar");
    declare_parameter("px4_local_position_topic", "/fmu/out/vehicle_local_position_v1");
    declare_parameter("px4_vehicle_status_topic", "/fmu/out/vehicle_status_v4");
    declare_parameter("offboard_control_mode_topic", "/fmu/in/offboard_control_mode");
    declare_parameter("trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint");
    declare_parameter("vehicle_command_topic", "/fmu/in/vehicle_command");
    declare_parameter("status_topic", "/px4_bridge/status");
    declare_parameter("manual_yaw_topic", "/px4_bridge/manual_yaw");
    declare_parameter("clear_manual_yaw_topic", "/px4_bridge/clear_manual_yaw");
    declare_parameter("observe_only", true);
    declare_parameter("calibration_valid", false);
    declare_parameter("start_offboard_on_first_command", true);
    declare_parameter("arm_on_first_command", false);
    declare_parameter("prestream_seconds", 1.0);
    declare_parameter("publish_rate_hz", 20.0);
    declare_parameter("max_command_age_sec", 0.5);
    declare_parameter("planner_watchdog_timeout_sec", 0.5);
    declare_parameter("max_abs_position_m", 10.0);
    declare_parameter("max_velocity_mps", 5.0);
    declare_parameter("max_acceleration_mps2", 10.0);
    declare_parameter("max_yaw_rate_rps", 3.0);
    declare_parameter("startup_takeoff_altitude_m", 0.0);
    declare_parameter("startup_takeoff_tolerance_m", 0.08);
    declare_parameter("startup_takeoff_duration_sec", 2.5);
    declare_parameter("startup_takeoff_velocity_tolerance_mps", 0.12);
    declare_parameter("startup_takeoff_stable_samples", 10);
    declare_parameter("transform_translation", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter("transform_rotation_row_major", std::vector<double>{
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0});

    planner_frame_ = get_parameter("planner_frame").as_string();
    observe_only_ = get_parameter("observe_only").as_bool();
    calibration_valid_ = get_parameter("calibration_valid").as_bool();
    start_offboard_ = get_parameter("start_offboard_on_first_command").as_bool();
    arm_on_first_ = get_parameter("arm_on_first_command").as_bool();
    prestream_seconds_ = get_parameter("prestream_seconds").as_double();
    max_command_age_sec_ = get_parameter("max_command_age_sec").as_double();
    planner_watchdog_timeout_sec_ = get_parameter("planner_watchdog_timeout_sec").as_double();
    if (!std::isfinite(planner_watchdog_timeout_sec_) || planner_watchdog_timeout_sec_ <= 0.0)
      throw std::runtime_error("planner watchdog timeout must be positive");
    max_abs_position_m_ = get_parameter("max_abs_position_m").as_double();
    max_velocity_mps_ = get_parameter("max_velocity_mps").as_double();
    max_acceleration_mps2_ = get_parameter("max_acceleration_mps2").as_double();
    max_yaw_rate_rps_ = get_parameter("max_yaw_rate_rps").as_double();
    startup_takeoff_altitude_m_ = get_parameter("startup_takeoff_altitude_m").as_double();
    startup_takeoff_tolerance_m_ = get_parameter("startup_takeoff_tolerance_m").as_double();
    startup_takeoff_duration_sec_ = get_parameter("startup_takeoff_duration_sec").as_double();
    startup_takeoff_velocity_tolerance_mps_ =
      get_parameter("startup_takeoff_velocity_tolerance_mps").as_double();
    startup_takeoff_required_samples_ =
      get_parameter("startup_takeoff_stable_samples").as_int();
    if (!std::isfinite(startup_takeoff_altitude_m_) || startup_takeoff_altitude_m_ < 0.0 ||
      !std::isfinite(startup_takeoff_tolerance_m_) || startup_takeoff_tolerance_m_ <= 0.0 ||
      !std::isfinite(startup_takeoff_velocity_tolerance_mps_) ||
      startup_takeoff_velocity_tolerance_mps_ <= 0.0 || startup_takeoff_duration_sec_ <= 0.0 ||
      startup_takeoff_required_samples_ < 1)
    {
      throw std::runtime_error("startup takeoff altitude and tolerance must be valid");
    }
    load_transform();

    const auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    command_sub_ = create_subscription<PositionCommand>(
      get_parameter("planner_command_topic").as_string(), rclcpp::QoS(10),
      std::bind(&PlannerPx4Bridge::command_callback, this, std::placeholders::_1));
    emergency_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/planning/emergency_stop", rclcpp::QoS(10),
      std::bind(&PlannerPx4Bridge::emergency_callback, this, std::placeholders::_1));
    manual_yaw_sub_ = create_subscription<std_msgs::msg::Float64>(
      get_parameter("manual_yaw_topic").as_string(), rclcpp::QoS(10),
      std::bind(&PlannerPx4Bridge::manual_yaw_callback, this, std::placeholders::_1));
    clear_manual_yaw_sub_ = create_subscription<std_msgs::msg::Empty>(
      get_parameter("clear_manual_yaw_topic").as_string(), rclcpp::QoS(10),
      std::bind(&PlannerPx4Bridge::clear_manual_yaw_callback, this, std::placeholders::_1));
    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      get_parameter("px4_local_position_topic").as_string(), px4_qos,
      std::bind(&PlannerPx4Bridge::local_position_callback, this, std::placeholders::_1));
    vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      get_parameter("px4_vehicle_status_topic").as_string(), px4_qos,
      std::bind(&PlannerPx4Bridge::vehicle_status_callback, this, std::placeholders::_1));
    mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      get_parameter("offboard_control_mode_topic").as_string(), px4_qos);
    setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      get_parameter("trajectory_setpoint_topic").as_string(), px4_qos);
    vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      get_parameter("vehicle_command_topic").as_string(), px4_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("status_topic").as_string(), rclcpp::QoS(10));

    const double publish_rate = std::max(1.0, get_parameter("publish_rate_hz").as_double());
    // PX4 OffboardControlMode is a communication watchdog, so it must be
    // published in real time. Do not attach this timer to Gazebo /clock:
    // rendering load, pausing, or a reduced real-time factor would otherwise
    // create heartbeat gaps and make PX4 leave Offboard. The setpoint itself
    // remains the latest command generated by the planner's simulation clock.
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate),
      std::bind(&PlannerPx4Bridge::timer_callback, this));
    publish_status("waiting_for_px4_local_position_and_planner_command");
  }

private:
  void load_transform()
  {
    const auto translation = get_parameter("transform_translation").as_double_array();
    const auto rotation = get_parameter("transform_rotation_row_major").as_double_array();
    if (translation.size() != 3 || rotation.size() != 9) {
      throw std::runtime_error("transform must contain 3 translation and 9 rotation values");
    }
    std::copy(translation.begin(), translation.end(), translation_.begin());
    std::copy(rotation.begin(), rotation.end(), rotation_.begin());
    if (!finite(translation_)) {
      throw std::runtime_error("transform translation must be finite");
    }
    for (double value : rotation_) {
      if (!std::isfinite(value)) {
        throw std::runtime_error("transform rotation must be finite");
      }
    }
    for (size_t row = 0; row < 3; ++row) {
      for (size_t col = 0; col < 3; ++col) {
        double dot = 0.0;
        for (size_t k = 0; k < 3; ++k) {
          dot += rotation_[row * 3 + k] * rotation_[col * 3 + k];
        }
        if (std::abs(dot - (row == col ? 1.0 : 0.0)) > 1e-4) {
          throw std::runtime_error("transform rotation must be orthonormal");
        }
      }
    }
    const double determinant =
      rotation_[0] * (rotation_[4] * rotation_[8] - rotation_[5] * rotation_[7]) -
      rotation_[1] * (rotation_[3] * rotation_[8] - rotation_[5] * rotation_[6]) +
      rotation_[2] * (rotation_[3] * rotation_[7] - rotation_[4] * rotation_[6]);
    if (std::abs(determinant - 1.0) > 1e-4) {
      throw std::runtime_error("transform rotation must be a proper rotation");
    }
  }

  void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    if (!msg->xy_valid || !msg->z_valid || !finite(Vector3{msg->x, msg->y, msg->z})) {
      return;
    }
    local_position_ = {msg->x, msg->y, msg->z};
    px4_timestamp_ = msg->timestamp;
    px4_heading_ = std::isfinite(msg->heading) ? msg->heading : 0.0;
    if (!startup_takeoff_initialized_ && startup_takeoff_altitude_m_ > 0.0 &&
      !observe_only_ && calibration_valid_)
    {
      takeoff_origin_ = {msg->x, msg->y, msg->z};
      takeoff_target_ = takeoff_origin_;
      takeoff_target_[2] -= startup_takeoff_altitude_m_;
      Setpoint hold;
      hold.position = takeoff_origin_;
      hold.yaw = px4_heading_;
      latest_setpoint_ = hold;
      first_command_time_ = std::chrono::steady_clock::now();
      startup_takeoff_initialized_ = true;
      publish_status("startup_takeoff_hold_initialized_waiting_for_offboard_and_arm");
    }
    if (startup_takeoff_initialized_ && !startup_takeoff_complete_ && latest_setpoint_) {
      const double horizontal_speed = std::hypot(msg->vx, msg->vy);
      const double takeoff_elapsed = startup_takeoff_start_time_ ?
        (now() - *startup_takeoff_start_time_).seconds() : -1.0;
      const bool profile_finished = takeoff_elapsed >= startup_takeoff_duration_sec_;
      const bool at_altitude = std::abs(msg->z - takeoff_target_[2]) <=
        startup_takeoff_tolerance_m_;
      const bool settled = msg->v_xy_valid && msg->v_z_valid &&
        std::isfinite(horizontal_speed) && std::isfinite(msg->vz) &&
        horizontal_speed <= startup_takeoff_velocity_tolerance_mps_ &&
        std::abs(msg->vz) <= startup_takeoff_velocity_tolerance_mps_;
      startup_takeoff_stable_samples_ = profile_finished && at_altitude && settled ?
        startup_takeoff_stable_samples_ + 1 : 0;
      if (startup_takeoff_stable_samples_ >= startup_takeoff_required_samples_) {
        startup_takeoff_complete_ = true;
        publish_status("startup_takeoff_reached_target_waiting_for_planner_goal");
      }
    }
    const std::array<uint8_t, 3> counters{
      msg->xy_reset_counter, msg->z_reset_counter, msg->heading_reset_counter};
    if (reset_counters_ && counters != *reset_counters_) {
      reset_blocked_ = true;
      Setpoint hold;
      hold.position = *local_position_;
      hold.yaw = std::isfinite(msg->heading) ? msg->heading : 0.0;
      latest_setpoint_ = hold;
      publish_status("blocked_px4_estimator_reset_holding_current_px4_position");
    }
    reset_counters_ = counters;
  }

  void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
  {
    vehicle_status_ = *msg;
  }

  void manual_yaw_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (!std::isfinite(msg->data)) {
      publish_status("rejected_non_finite_manual_yaw");
      return;
    }
    manual_yaw_ = msg->data;
    publish_status("manual_yaw_enabled:lidar_rad=" + std::to_string(*manual_yaw_));
  }

  void clear_manual_yaw_callback(const std_msgs::msg::Empty::SharedPtr)
  {
    manual_yaw_.reset();
    publish_status("manual_yaw_cleared_using_planner_yaw");
  }

  void latch_px4_hold(const std::string & why)
  {
    if (!local_position_) {
      publish_status("cannot_hold_px4_no_valid_local_position:" + why);
      return;
    }
    Setpoint hold;
    hold.position = *local_position_;  // Native PX4 NED coordinates; no transform involved.
    hold.yaw = px4_heading_;
    latest_setpoint_ = hold;
    bridge_emergency_hold_ = true;
    publish_status("px4_position_hold_latched:" + why);
  }

  void emergency_callback(const std_msgs::msg::Empty::SharedPtr)
  {
    if (!bridge_emergency_hold_) {
      held_trajectory_id_ = last_trajectory_id_.value_or(0);
      latch_px4_hold("planner_emergency_stop");
    }
  }

  void command_callback(const PositionCommand::SharedPtr msg)
  {
    if (startup_takeoff_initialized_ && !startup_takeoff_complete_) {
      if (!takeoff_rejection_reported_) {
        publish_status("rejected_planner_command_while_startup_takeoff_active");
        takeoff_rejection_reported_ = true;
      }
      return;
    }
    takeoff_rejection_reported_ = false;
    if (reset_blocked_) {
      publish_status("rejected_command_after_px4_estimator_reset_restart_and_recalibrate");
      return;
    }
    if (msg->header.frame_id != planner_frame_) {
      publish_status("rejected_frame:" +
        (msg->header.frame_id.empty() ? std::string("<empty>") : msg->header.frame_id) +
        ":expected:" + planner_frame_);
      return;
    }
    if (bridge_emergency_hold_ && held_trajectory_id_ &&
        msg->trajectory_id <= *held_trajectory_id_) {
      return;  // No stale/latched trajectory can overwrite the PX4 hold.
    }
    if (msg->trajectory_flag != PositionCommand::TRAJECTORY_STATUS_READY) {
      publish_status("rejected_trajectory_status:" + std::to_string(msg->trajectory_flag));
      return;
    }
    const auto now = get_clock()->now();
    const int64_t stamp_ns =
      static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL + msg->header.stamp.nanosec;
    if (stamp_ns <= 0) {
      publish_status("rejected_missing_command_timestamp");
      return;
    }
    const double age = static_cast<double>(now.nanoseconds() - stamp_ns) / 1e9;
    if (age < -0.05 || age > max_command_age_sec_) {
      std::ostringstream status;
      status << "rejected_stale_command:age_sec=" << age;
      publish_status(status.str());
      return;
    }

    const Vector3 position{msg->position.x, msg->position.y, msg->position.z};
    const Vector3 velocity{msg->velocity.x, msg->velocity.y, msg->velocity.z};
    const Vector3 acceleration{msg->acceleration.x, msg->acceleration.y, msg->acceleration.z};
    if (!finite(position) || !finite(velocity) || !finite(acceleration)) {
      publish_status("rejected_non_finite_command");
      return;
    }
    if (!std::isfinite(msg->yaw) || !std::isfinite(msg->yaw_dot)) {
      publish_status("rejected_non_finite_yaw_or_yaw_rate");
      return;
    }

    Setpoint output;
    output.position = rotate(rotation_, position);
    for (size_t i = 0; i < output.position.size(); ++i) {
      output.position[i] += translation_[i];
    }
    output.velocity = rotate(rotation_, velocity);
    output.acceleration = rotate(rotation_, acceleration);
    const double planner_yaw = manual_yaw_.value_or(msg->yaw);
    const auto heading = rotate(rotation_, {std::cos(planner_yaw), std::sin(planner_yaw), 0.0});
    if (std::hypot(heading[0], heading[1]) < 1e-6) {
      publish_status("rejected_yaw_transform_is_singular");
      return;
    }
    output.yaw = std::atan2(heading[1], heading[0]);
    if (manual_yaw_) {
      output.yaw_rate = 0.0;
    } else {
      const auto angular_velocity = rotate(rotation_, {0.0, 0.0, msg->yaw_dot});
      output.yaw_rate = angular_velocity[2];
    }
    if (std::any_of(output.position.begin(), output.position.end(), [this](double value) {
        return std::abs(value) > max_abs_position_m_;
      }))
    {
      publish_status("rejected_position_out_of_configured_bounds");
      return;
    }
    if (norm(output.velocity) > max_velocity_mps_) {
      publish_status("rejected_velocity_out_of_configured_bounds");
      return;
    }
    if (norm(output.acceleration) > max_acceleration_mps2_) {
      publish_status("rejected_acceleration_out_of_configured_bounds");
      return;
    }
    if (std::abs(output.yaw_rate) > max_yaw_rate_rps_) {
      publish_status("rejected_yaw_rate_out_of_configured_bounds");
      return;
    }

    // Recovery only occurs after a new, fully validated command on a newer
    // trajectory id. We keep all existing frame and coordinate checks intact.
    last_trajectory_id_ = msg->trajectory_id;
    latest_command_time_ = std::chrono::steady_clock::now();
    bridge_emergency_hold_ = false;
    held_trajectory_id_.reset();
    latest_setpoint_ = output;
    if (!first_command_time_) {
      first_command_time_ = std::chrono::steady_clock::now();
      offboard_requested_ = false;
      arm_requested_ = false;
    }
    std::ostringstream status;
    status << (observe_only_ ? "observe_only_configured_transform" :
      (start_offboard_ ? "first_command_received_waiting_for_prestream" :
      "publishing_setpoints_without_mode_change"))
           << (calibration_valid_ ? ":px4_position=[" : ":unvalidated_position=[")
           << output.position[0] << "," << output.position[1] << ","
           << output.position[2] << "]:yaw=" << output.yaw << ":yaw_rate=" << output.yaw_rate
           << (manual_yaw_ ? ":manual_yaw=true" : ":manual_yaw=false");
    const auto steady_now = std::chrono::steady_clock::now();
    if (!last_planner_status_time_ ||
      std::chrono::duration<double>(steady_now - *last_planner_status_time_).count() >= 1.0)
    {
      publish_status(status.str());
      last_planner_status_time_ = steady_now;
    }
  }

  static double norm(const Vector3 & vector)
  {
    return std::sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
  }

  void timer_callback()
  {
    if (!local_position_) {
      publish_status("waiting_for_valid_px4_local_position");
      return;
    }
    update_startup_takeoff_setpoint();
    if (reset_blocked_ && !first_command_time_) {
      return;
    }
    if (!latest_setpoint_ || observe_only_) {
      return;
    }
    if (!calibration_valid_ && !reset_blocked_) {
      publish_status("blocked_calibration_valid_is_false");
      return;
    }

    const auto steady_now_watchdog = std::chrono::steady_clock::now();
    if (!bridge_emergency_hold_ && latest_command_time_ &&
        std::chrono::duration<double>(steady_now_watchdog - *latest_command_time_).count() >
            planner_watchdog_timeout_sec_) {
      held_trajectory_id_ = last_trajectory_id_.value_or(0);
      latch_px4_hold("planner_command_watchdog_timeout");
    }

    const uint64_t timestamp = px4_timestamp_;
    px4_msgs::msg::OffboardControlMode mode;
    mode.timestamp = timestamp;
    mode.position = true;
    mode_pub_->publish(mode);

    px4_msgs::msg::TrajectorySetpoint setpoint;
    setpoint.timestamp = timestamp;
    setpoint.position = to_float_array(latest_setpoint_->position);
    setpoint.velocity = to_float_array(latest_setpoint_->velocity);
    setpoint.acceleration = to_float_array(latest_setpoint_->acceleration);
    setpoint.jerk = {NAN, NAN, NAN};
    setpoint.yaw = static_cast<float>(latest_setpoint_->yaw);
    setpoint.yawspeed = static_cast<float>(latest_setpoint_->yaw_rate);
    setpoint_pub_->publish(setpoint);
    // Never begin/re-arm automatically while a safety hold is latched.
    if (bridge_emergency_hold_ || reset_blocked_ || !start_offboard_) {
      return;
    }
    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - *first_command_time_).count();
    if (elapsed < prestream_seconds_) {
      return;
    }
    if (!vehicle_status_) {
      publish_status("waiting_for_px4_vehicle_status_before_mode_request");
      return;
    }
    const bool disarmed = vehicle_status_->arming_state ==
      px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;
    if (!vehicle_status_->pre_flight_checks_pass || (vehicle_status_->failsafe && !disarmed)) {
      publish_status("mode_request_waiting_for_px4_preflight_or_armed_failsafe");
      return;
    }

    const auto steady_now = std::chrono::steady_clock::now();
    if (vehicle_status_->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
      offboard_requested_ = true;
    }
    if (!offboard_requested_) {
      if (command_retry_due(steady_now)) {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
        last_vehicle_command_time_ = steady_now;
        publish_status("offboard_mode_requested_waiting_for_px4_confirmation");
      }
      return;
    }
    if (vehicle_status_->nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
      publish_status("px4_left_offboard_mode_bridge_will_not_override_current_px4_mode");
      return;
    }

    if (arm_on_first_ &&
      vehicle_status_->arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED &&
      command_retry_due(steady_now))
    {
      if (!vehicle_status_->pre_flight_checks_pass || vehicle_status_->failsafe) {
        publish_status("arming_waiting_for_px4_preflight_pass_and_no_failsafe");
        return;
      }
      publish_vehicle_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);
      arm_requested_ = true;
      last_vehicle_command_time_ = steady_now;
      publish_status("offboard_confirmed_arm_requested_waiting_for_px4_confirmation");
      return;
    }
    if (vehicle_status_->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
      if (startup_takeoff_initialized_ && !startup_takeoff_start_time_) {
        startup_takeoff_start_time_ = now();
        publish_status("startup_takeoff_profile_started");
      }
      publish_status("offboard_active_vehicle_armed_tracking_latest_planner_setpoint");
    } else {
      publish_status("offboard_active_vehicle_disarmed_tracking_latest_planner_setpoint");
    }
  }

  void update_startup_takeoff_setpoint()
  {
    if (!startup_takeoff_initialized_ || startup_takeoff_complete_ || !startup_takeoff_start_time_) {
      return;
    }

    const double elapsed = std::max(0.0, (now() - *startup_takeoff_start_time_).seconds());
    const double u = std::clamp(elapsed / startup_takeoff_duration_sec_, 0.0, 1.0);
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double u4 = u3 * u;
    const double u5 = u4 * u;
    const double blend = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
    const double blend_rate = (30.0 * u2 - 60.0 * u3 + 30.0 * u4) /
      startup_takeoff_duration_sec_;
    const double blend_accel = (60.0 * u - 180.0 * u2 + 120.0 * u3) /
      (startup_takeoff_duration_sec_ * startup_takeoff_duration_sec_);
    const double delta_z = takeoff_target_[2] - takeoff_origin_[2];
    latest_setpoint_->position = takeoff_origin_;
    latest_setpoint_->position[2] = takeoff_origin_[2] + delta_z * blend;
    latest_setpoint_->velocity = {0.0, 0.0, delta_z * blend_rate};
    latest_setpoint_->acceleration = {0.0, 0.0, delta_z * blend_accel};
  }

  bool command_retry_due(const std::chrono::steady_clock::time_point now) const
  {
    return !last_vehicle_command_time_ ||
           std::chrono::duration<double>(now - *last_vehicle_command_time_).count() >= 1.0;
  }

  static std::array<float, 3> to_float_array(const Vector3 & vector)
  {
    return {static_cast<float>(vector[0]), static_cast<float>(vector[1]),
      static_cast<float>(vector[2])};
  }

  void publish_vehicle_command(uint16_t command, float param1, float param2)
  {
    px4_msgs::msg::VehicleCommand msg;
    msg.timestamp = px4_timestamp_;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    vehicle_command_pub_->publish(msg);
  }

  void publish_status(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
  }

  std::string planner_frame_;
  bool observe_only_{true};
  bool calibration_valid_{false};
  bool start_offboard_{true};
  bool arm_on_first_{false};
  bool offboard_requested_{false};
  bool arm_requested_{false};
  bool reset_blocked_{false};
  double prestream_seconds_{1.0};
  double max_command_age_sec_{0.5};
  double planner_watchdog_timeout_sec_{0.5};
  bool bridge_emergency_hold_{false};
  std::optional<int> held_trajectory_id_;
  std::optional<int> last_trajectory_id_;
  std::optional<std::chrono::steady_clock::time_point> latest_command_time_;
  double max_abs_position_m_{10.0};
  double max_velocity_mps_{5.0};
  double max_acceleration_mps2_{10.0};
  double max_yaw_rate_rps_{3.0};
  double startup_takeoff_altitude_m_{0.0};
  double startup_takeoff_tolerance_m_{0.08};
  double startup_takeoff_duration_sec_{2.5};
  double startup_takeoff_velocity_tolerance_mps_{0.12};
  int startup_takeoff_required_samples_{10};
  int startup_takeoff_stable_samples_{0};
  double px4_heading_{0.0};
  bool startup_takeoff_initialized_{false};
  bool startup_takeoff_complete_{false};
  bool takeoff_rejection_reported_{false};
  Vector3 translation_{};
  Matrix3 rotation_{};
  Vector3 takeoff_origin_{};
  Vector3 takeoff_target_{};
  std::optional<Vector3> local_position_;
  std::optional<Setpoint> latest_setpoint_;
  std::optional<double> manual_yaw_;
  std::optional<std::array<uint8_t, 3>> reset_counters_;
  std::optional<std::chrono::steady_clock::time_point> first_command_time_;
  std::optional<std::chrono::steady_clock::time_point> last_vehicle_command_time_;
  std::optional<std::chrono::steady_clock::time_point> last_planner_status_time_;
  std::optional<rclcpp::Time> startup_takeoff_start_time_;
  std::optional<px4_msgs::msg::VehicleStatus> vehicle_status_;
  uint64_t px4_timestamp_{0};
  rclcpp::Subscription<PositionCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr emergency_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr manual_yaw_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr clear_manual_yaw_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PlannerPx4Bridge>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("px4_bridge"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
