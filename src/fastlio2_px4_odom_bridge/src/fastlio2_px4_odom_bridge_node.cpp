#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
using Matrix3 = Eigen::Matrix3d;
using Vector3 = Eigen::Vector3d;

bool finite(const Vector3 & v)
{
  return v.array().isFinite().all();
}

double finite_or(double value, double fallback)
{
  return std::isfinite(value) && value >= 0.0 ? value : fallback;
}
}  // namespace

class Fastlio2Px4OdomBridge final : public rclcpp::Node
{
public:
  Fastlio2Px4OdomBridge()
  : Node("fastlio2_px4_odom_bridge")
  {
    declare_parameter("input_topic", "/fastlio2/lio_odom");
    declare_parameter("output_topic", "/fmu/in/vehicle_visual_odometry");
    declare_parameter("input_frame", "lidar");
    declare_parameter("reject_frame_mismatch", true);
    declare_parameter("timestamp_mode", "now");  // now or odom_stamp
    declare_parameter("position_variance_floor", 0.01);
    declare_parameter("orientation_variance_floor", 0.01);
    declare_parameter("velocity_variance_floor", 0.04);
    declare_parameter("transform_translation", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter("world_rotation_row_major", std::vector<double>{
      0.0, 1.0, 0.0,
      1.0, 0.0, 0.0,
      0.0, 0.0, -1.0});
    // FAST-LIO consumes the Gazebo IMU after the MID-360 plugin removes the
    // sensor mount yaw, leaving the X500 base_link convention (FLU). PX4 uses
    // FRD, so FLU -> FRD is a pi rotation about X.
    declare_parameter("body_to_frd_row_major", std::vector<double>{
      1.0, 0.0, 0.0,
      0.0, -1.0, 0.0,
      0.0, 0.0, -1.0});

    input_frame_ = get_parameter("input_frame").as_string();
    reject_frame_mismatch_ = get_parameter("reject_frame_mismatch").as_bool();
    timestamp_mode_ = get_parameter("timestamp_mode").as_string();
    position_floor_ = get_parameter("position_variance_floor").as_double();
    orientation_floor_ = get_parameter("orientation_variance_floor").as_double();
    velocity_floor_ = get_parameter("velocity_variance_floor").as_double();
    translation_ = vector3_parameter("transform_translation");
    world_rotation_ = rotation_parameter("world_rotation_row_major");
    body_to_frd_ = rotation_parameter("body_to_frd_row_major");
    if (timestamp_mode_ != "now" && timestamp_mode_ != "odom_stamp") {
      throw std::runtime_error("timestamp_mode must be now or odom_stamp");
    }
    if (!std::isfinite(position_floor_) || !std::isfinite(orientation_floor_) ||
      !std::isfinite(velocity_floor_) || position_floor_ < 0.0 ||
      orientation_floor_ < 0.0 || velocity_floor_ < 0.0)
    {
      throw std::runtime_error("variance floors must be finite and non-negative");
    }

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("input_topic").as_string(), qos,
      std::bind(&Fastlio2Px4OdomBridge::odom_callback, this, std::placeholders::_1));
    odom_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
      get_parameter("output_topic").as_string(), qos);
    RCLCPP_INFO(get_logger(), "FAST-LIO odometry bridge: %s -> %s",
      get_parameter("input_topic").as_string().c_str(),
      get_parameter("output_topic").as_string().c_str());
  }

private:
  Vector3 vector3_parameter(const std::string & name)
  {
    const auto values = get_parameter(name).as_double_array();
    if (values.size() != 3) {
      throw std::runtime_error(name + " must contain 3 values");
    }
    return Vector3(values[0], values[1], values[2]);
  }

  Matrix3 rotation_parameter(const std::string & name)
  {
    const auto values = get_parameter(name).as_double_array();
    if (values.size() != 9) {
      throw std::runtime_error(name + " must contain 9 values");
    }
    Matrix3 result;
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        result(row, col) = values[static_cast<size_t>(row * 3 + col)];
      }
    }
    if (!result.allFinite() || (result * result.transpose() - Matrix3::Identity()).norm() > 1e-4 ||
      std::abs(result.determinant() - 1.0) > 1e-4)
    {
      throw std::runtime_error(name + " must be a proper rotation matrix");
    }
    return result;
  }

  uint64_t timestamp_us(const nav_msgs::msg::Odometry & msg) const
  {
    if (timestamp_mode_ == "odom_stamp") {
      const int64_t ns = static_cast<int64_t>(msg.header.stamp.sec) * 1000000000LL +
        static_cast<int64_t>(msg.header.stamp.nanosec);
      if (ns > 0) {
        return static_cast<uint64_t>(ns / 1000);
      }
    }
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  }

  static float variance(const std::array<double, 36> & covariance, size_t index, double floor)
  {
    return static_cast<float>(finite_or(covariance[index], floor) < floor ?
      floor : finite_or(covariance[index], floor));
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (reject_frame_mismatch_ && msg->header.frame_id != input_frame_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Rejecting odometry frame '%s'; expected '%s'", msg->header.frame_id.c_str(), input_frame_.c_str());
      return;
    }
    const auto & p = msg->pose.pose.position;
    const auto & q_msg = msg->pose.pose.orientation;
    const Vector3 position_in(p.x, p.y, p.z);
    const Vector3 velocity_body(
      msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    const Vector3 angular_body(
      msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    Eigen::Quaterniond q_in(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
    if (!finite(position_in) || !finite(velocity_body) || !finite(angular_body) ||
      !std::isfinite(q_in.norm()) || q_in.norm() < 1e-6)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Rejecting non-finite FAST-LIO odometry");
      return;
    }
    q_in.normalize();

    // q_in maps FAST-LIO child/body axes into the lidar world. The output
    // quaternion must map PX4 FRD body axes into PX4 NED world axes.
    const Matrix3 body_to_lidar = q_in.toRotationMatrix();
    const Matrix3 frd_to_body = body_to_frd_.transpose();
    const Matrix3 ned_from_frd = world_rotation_ * body_to_lidar * frd_to_body;
    const Vector3 position_ned = world_rotation_ * position_in + translation_;
    const Vector3 velocity_ned = world_rotation_ * body_to_lidar * velocity_body;
    const Vector3 angular_frd = body_to_frd_ * angular_body;
    if (!finite(position_ned) || !finite(velocity_ned) || !finite(angular_frd)) {
      return;
    }

    px4_msgs::msg::VehicleOdometry out;
    out.timestamp = timestamp_us(*msg);
    out.timestamp_sample = out.timestamp;
    out.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    out.position = {static_cast<float>(position_ned.x()), static_cast<float>(position_ned.y()),
      static_cast<float>(position_ned.z())};
    const Eigen::Quaterniond q_out(ned_from_frd);
    out.q = {static_cast<float>(q_out.w()), static_cast<float>(q_out.x()),
      static_cast<float>(q_out.y()), static_cast<float>(q_out.z())};
    out.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
    out.velocity = {static_cast<float>(velocity_ned.x()), static_cast<float>(velocity_ned.y()),
      static_cast<float>(velocity_ned.z())};
    out.angular_velocity = {static_cast<float>(angular_frd.x()), static_cast<float>(angular_frd.y()),
      static_cast<float>(angular_frd.z())};
    out.position_variance = {
      variance(msg->pose.covariance, 0, position_floor_),
      variance(msg->pose.covariance, 7, position_floor_),
      variance(msg->pose.covariance, 14, position_floor_)};
    out.orientation_variance = {
      variance(msg->pose.covariance, 21, orientation_floor_),
      variance(msg->pose.covariance, 28, orientation_floor_),
      variance(msg->pose.covariance, 35, orientation_floor_)};
    out.velocity_variance = {
      variance(msg->twist.covariance, 0, velocity_floor_),
      variance(msg->twist.covariance, 7, velocity_floor_),
      variance(msg->twist.covariance, 14, velocity_floor_)};
    // FAST-LIO does not expose a reset counter in nav_msgs/Odometry. Keep a
    // stable counter until a future discontinuity detector is added.
    out.reset_counter = reset_counter_;
    out.quality = 100;
    odom_pub_->publish(out);
  }

  std::string input_frame_;
  std::string timestamp_mode_;
  bool reject_frame_mismatch_{true};
  double position_floor_{0.01};
  double orientation_floor_{0.01};
  double velocity_floor_{0.04};
  Vector3 translation_{};
  Matrix3 world_rotation_{Matrix3::Identity()};
  Matrix3 body_to_frd_{Matrix3::Identity()};
  uint8_t reset_counter_{0};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Fastlio2Px4OdomBridge>());
  rclcpp::shutdown();
  return 0;
}
