#include "plan_manage/kino_replan_fsm.h"
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/empty.hpp>
#include <cmath>

namespace fast_planner {

KinoReplanFSM::KinoReplanFSM() {
   std::cout << "KinoReplanFSM hit ! " << std::endl;
}

KinoReplanFSM::~KinoReplanFSM() {
  std::cout << "KinoReplanFSM ended ! " << std::endl;
}

void KinoReplanFSM::init(std::shared_ptr<FastPlanner> nh) {
  current_wp_  = 0;
  node_ = nh;
  exec_state_  = FSM_EXEC_STATE::INIT;
  have_target_ = false;
  have_odom_   = false;

  nh->declare_parameter("fsm/flight_type", -1);
  nh->declare_parameter("fsm/thresh_replan", -1.0);
  nh->declare_parameter("fsm/thresh_no_replan", -1.0);
  nh->declare_parameter("fsm/replan_retry_interval", 0.25);
  nh->declare_parameter("fsm/tracking_error_replan", 0.45);
  nh->declare_parameter("fsm/tracking_error_emergency", 0.9);
  nh->declare_parameter("fsm/odom_timeout_sec", 0.5);
  nh->declare_parameter("fsm/waypoint_num", -1);
  // /goal_pose is the bridge/manual_goal contract.  Launch files can still
  // override this for a visualization or multi-UAV namespace.
  nh->declare_parameter("goal_topic", "/goal_pose");

  nh->get_parameter("fsm/flight_type", target_type_);
  nh->get_parameter("fsm/thresh_replan", replan_thresh_);
  nh->get_parameter("fsm/thresh_no_replan", no_replan_thresh_);
  nh->get_parameter("fsm/replan_retry_interval", replan_retry_interval_);
  nh->get_parameter("fsm/tracking_error_replan", tracking_error_replan_);
  nh->get_parameter("fsm/tracking_error_emergency", tracking_error_emergency_);
  nh->get_parameter("fsm/odom_timeout_sec", odom_timeout_sec_);
  if (!std::isfinite(odom_timeout_sec_) || odom_timeout_sec_ <= 0.0)
    odom_timeout_sec_ = 0.5;
  if (!std::isfinite(tracking_error_replan_) || tracking_error_replan_ <= 0.0 ||
      !std::isfinite(tracking_error_emergency_) ||
      tracking_error_emergency_ <= tracking_error_replan_) {
    tracking_error_replan_ = 0.45;
    tracking_error_emergency_ = 0.9;
    RCLCPP_WARN(nh->get_logger(), "Invalid tracking error limits; using 0.45 / 0.9 m");
  }
  if (!std::isfinite(replan_retry_interval_) || replan_retry_interval_ < 0.0) {
    RCLCPP_WARN(nh->get_logger(),
                "fsm/replan_retry_interval must be finite and nonnegative; using 0.25 s");
    replan_retry_interval_ = 0.25;
  }
  nh->get_parameter("fsm/waypoint_num", waypoint_num_);

  //waypoints_.resize(waypoint_num_);
  nh->declare_parameter("fsm/waypoint0_x", -1.0);
  nh->declare_parameter("fsm/waypoint0_y", -1.0);
  nh->declare_parameter("fsm/waypoint0_z", -1.0);
  nh->declare_parameter("fsm/waypoint1_x", -1.0);
  nh->declare_parameter("fsm/waypoint1_y", -1.0);
  nh->declare_parameter("fsm/waypoint1_z", -1.0);
  nh->declare_parameter("fsm/waypoint2_x", -1.0);
  nh->declare_parameter("fsm/waypoint2_y", -1.0);
  nh->declare_parameter("fsm/waypoint2_z", -1.0);

  nh->get_parameter("fsm/waypoint0_x", waypoints_[0][0]);
  nh->get_parameter("fsm/waypoint0_y", waypoints_[0][1]);
  nh->get_parameter("fsm/waypoint0_z", waypoints_[0][2]);

  nh->get_parameter("fsm/waypoint1_x", waypoints_[1][0]);
  nh->get_parameter("fsm/waypoint1_y", waypoints_[1][1]);
  nh->get_parameter("fsm/waypoint1_z", waypoints_[1][2]);

  nh->get_parameter("fsm/waypoint2_x", waypoints_[2][0]);
  nh->get_parameter("fsm/waypoint2_y", waypoints_[2][1]);
  nh->get_parameter("fsm/waypoint2_z", waypoints_[2][2]);

  //////////////////////// DOUBTS - NEXT 3 lines
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);
  nh->get_parameter_or("sdf_map/frame_id", planning_frame_, std::string("lidar"));
  visualization_.reset(new PlanningVisualization(nh));
  exec_timer_ = nh->create_wall_timer(std::chrono::milliseconds(10), std::bind(&KinoReplanFSM::execFSMCallback, this));
  safety_timer_ = nh->create_wall_timer(std::chrono::milliseconds(50), std::bind(&KinoReplanFSM::checkCollisionCallback, this));

  const auto goal_topic = nh->get_parameter("goal_topic").as_string();
  waypoint_sub_ = nh->create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic, rclcpp::QoS(10),
      std::bind(&KinoReplanFSM::waypointCallback, this, std::placeholders::_1));
  odom_sub_ = nh->create_subscription<nav_msgs::msg::Odometry>(
      "/odom_world", 1, std::bind(&KinoReplanFSM::odometryCallback, this, std::placeholders::_1));

  replan_pub_ = nh->create_publisher<std_msgs::msg::Empty>("/planning/replan", 10);
  emergency_stop_pub_ = nh->create_publisher<std_msgs::msg::Empty>("/planning/emergency_stop", 10);
  new_pub_ = nh->create_publisher<std_msgs::msg::Empty>("/planning/new", 10);
  bspline_pub_ = nh->create_publisher<quadrotor_msgs::msg::Bspline>("/planning/bspline", 10);
}

void KinoReplanFSM::waypointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (!msg->header.frame_id.empty() && msg->header.frame_id != planning_frame_) {
    RCLCPP_WARN(node_->get_logger(),
                "Ignoring goal in frame '%s'; planner frame is '%s'",
                msg->header.frame_id.c_str(), planning_frame_.c_str());
    return;
  }

  const auto& goal = msg->pose.position;
  if (!std::isfinite(goal.x) || !std::isfinite(goal.y) || !std::isfinite(goal.z)) {
    RCLCPP_WARN(node_->get_logger(), "Ignoring goal with non-finite coordinates");
    return;
  }

  if (msg->pose.position.z < -0.1) {
    return;
  }

  RCUTILS_LOG_INFO("Triggered!");
  trigger_ = true;

  if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {
    end_pt_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  } else if (target_type_ == TARGET_TYPE::PRESET_TARGET) {
    end_pt_(0) = waypoints_[current_wp_][0];
    end_pt_(1) = waypoints_[current_wp_][1];
    end_pt_(2) = waypoints_[current_wp_][2];
    current_wp_ = (current_wp_ + 1) % waypoint_num_;
  }

  std::cout << "DEBugging waypoints .....................................ok done ! ";
  std::cout << end_pt_ << std::endl;

  visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  end_vel_.setZero();
  have_target_ = true;

  if (exec_state_ == INIT || exec_state_ == WAIT_TARGET)
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
  else if (exec_state_ == EXEC_TRAJ)
    changeFSMExecState(REPLAN_TRAJ, "TRIG");
}

void KinoReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  if (!msg->header.frame_id.empty() && msg->header.frame_id != planning_frame_) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Ignoring odometry in frame '%s'; planner frame is '%s'",
      msg->header.frame_id.c_str(), planning_frame_.c_str());
    return;
  }

  const Eigen::Vector3d position(
      msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  const Eigen::Vector3d child_velocity(
      msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
  Eigen::Quaterniond orientation(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  if (!position.allFinite() || !child_velocity.allFinite() ||
      !std::isfinite(orientation.norm()) || orientation.norm() < 1e-6) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "Ignoring odometry with invalid pose, velocity, or orientation");
    return;
  }

  orientation.normalize();
  odom_pos_ = position;
  odom_orient_ = orientation;
  // FAST-LIO publishes Odometry twist in child_frame_id (body), while the
  // planner's dynamics and position are expressed in header.frame_id (lidar).
  odom_vel_ = odom_orient_ * child_velocity;

  last_odom_time_ = std::chrono::steady_clock::now();
  have_odom_ = true;
}

bool KinoReplanFSM::odometryFresh() const {
  return have_odom_ && std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_odom_time_).count() <= odom_timeout_sec_;
}

void KinoReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call) {
  std::vector<std::string> state_str = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ"};
  int pre_s = static_cast<int>(exec_state_);
  exec_state_ = new_state;
  // RCLCPP_INFO(this->get_logger(), "[%s]: from %s to %s", pos_call.c_str(), state_str[pre_s].c_str(), state_str[static_cast<int>(new_state)].c_str());
  RCUTILS_LOG_INFO("[%s]: from %s to %s", pos_call.c_str(), state_str[pre_s].c_str(), state_str[static_cast<int>(new_state)].c_str());
}

void KinoReplanFSM::emergencyStop(const char* reason) {
  RCLCPP_ERROR(node_->get_logger(), "Emergency position hold requested: %s", reason);
  emergency_stop_pub_->publish(std_msgs::msg::Empty{});
}

void KinoReplanFSM::printFSMExecState() {
  std::vector<std::string> state_str = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ"};
  //RCLCPP_INFO(this->get_logger(), "[FSM]: state: %s", state_str[static_cast<int>(exec_state_)].c_str());
  RCUTILS_LOG_INFO("[FSM]: state: %s", state_str[static_cast<int>(exec_state_)].c_str());
}

void KinoReplanFSM::execFSMCallback() {
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    printFSMExecState();
    if (!have_odom_) RCUTILS_LOG_INFO("no odom");
    if (!trigger_) RCUTILS_LOG_INFO("wait for goal");
    fsm_num = 0;
  }

  // Never seed a plan from indefinitely old FAST-LIO odometry.
  if (!odometryFresh()) {
    if (exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ) {
      emergencyStop("odometry timeout");
      changeFSMExecState(GEN_NEW_TRAJ, "ODOM_STALE");
    }
    return;
  }

  switch (exec_state_) {
    case INIT:{
      if (!have_odom_ || !trigger_) return;
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;}

    case WAIT_TARGET:{
      if (!have_target_) return;
      changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      break;}

    case GEN_NEW_TRAJ:{
      if (std::chrono::steady_clock::now() < next_plan_attempt_) return;
      if ((odom_pos_ - end_pt_).norm() < no_replan_thresh_) {
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "GOAL_REACHED");
        return;
      }
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();
      start_yaw_(0) = atan2(odom_orient_.toRotationMatrix()(1, 0), odom_orient_.toRotationMatrix()(0, 0));
      start_yaw_(1) = start_yaw_(2) = 0.0;
      if (callKinodynamicReplan()) {
        replan_pending_ = false;
        next_plan_attempt_ = std::chrono::steady_clock::time_point::min();
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        next_plan_attempt_ = std::chrono::steady_clock::now() +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(replan_retry_interval_));
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;}
      case EXEC_TRAJ: {
            /* determine if need to replan */
            LocalTrajData* info     = &planner_manager_->local_data_;
            rclcpp::Time time_now = node_->get_clock()->now();
            double         t_cur    = (time_now - info->start_time_).seconds();
            t_cur = std::clamp(t_cur, 0.0, info->duration_);

            if ((end_pt_ - odom_pos_).norm() < no_replan_thresh_) {
              have_target_ = false;
              changeFSMExecState(WAIT_TARGET, "GOAL_REACHED");
              return;
            }

            if (t_cur >= info->duration_ - 1e-2) {
              // The planned endpoint is not proof that the aircraft reached
              // it. Keep the goal and generate a new segment from live odometry.
              next_plan_attempt_ = std::chrono::steady_clock::time_point::min();
              changeFSMExecState(GEN_NEW_TRAJ, "TRAJ_END_BEFORE_GOAL");
              return;
            }

            const Eigen::Vector3d planned_pos = info->position_traj_.evaluateDeBoorT(t_cur);
            const double tracking_error = (odom_pos_ - planned_pos).norm();
            if (tracking_error >= tracking_error_emergency_) {
              emergencyStop("tracking error exceeds emergency threshold");
              changeFSMExecState(GEN_NEW_TRAJ, "TRACKING_ERROR_STOP");
            } else if (std::chrono::steady_clock::now() >= next_plan_attempt_ &&
                (replan_pending_ || tracking_error >= tracking_error_replan_ ||
                 (info->start_pos_ - planned_pos).norm() >= replan_thresh_)) {
              changeFSMExecState(REPLAN_TRAJ, "FSM");
            }
            break;
          }
    case REPLAN_TRAJ:{
      if (std::chrono::steady_clock::now() < next_plan_attempt_) return;
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();
      start_yaw_(0) = std::atan2(
          odom_orient_.toRotationMatrix()(1, 0), odom_orient_.toRotationMatrix()(0, 0));
      start_yaw_(1) = 0.0;
      start_yaw_(2) = 0.0;

      // Keep executing the validated old trajectory during normal replans.
      // Truncating it before search succeeds creates abrupt, unsafe setpoint jumps.
      bool success = callKinodynamicReplan();
      if (success) {
        replan_pending_ = false;
        next_plan_attempt_ = std::chrono::steady_clock::time_point::min();
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        replan_pending_ = true;
        next_plan_attempt_ = std::chrono::steady_clock::now() +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(replan_retry_interval_));
        LocalTrajData* info = &planner_manager_->local_data_;
        const double elapsed = (node_->get_clock()->now() - info->start_time_).seconds();
        if (elapsed < info->duration_ - 0.1) {
          // A failed search is not itself a flight emergency. Keep tracking
          // the active curve while the safety timer checks it, then retry.
          RCLCPP_WARN(node_->get_logger(),
                      "Replan failed; continuing the active trajectory and retrying in %.2f s",
                      replan_retry_interval_);
          changeFSMExecState(EXEC_TRAJ, "REPLAN_RETRY");
        } else {
          changeFSMExecState(GEN_NEW_TRAJ, "REPLAN_RETRY_AT_SEGMENT_END");
        }
      }
      break;
    }
  }
}

void KinoReplanFSM::checkCollisionCallback() {
  LocalTrajData* info = &planner_manager_->local_data_;

  if (!odometryFresh()) return;  // execFSMCallback handles timeouts.
  if (!planner_manager_->edt_environment_->sdf_map_->hasFreshPointCloud()) {
    if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                           "Point cloud is stale; requesting position hold");
      emergencyStop("point cloud timeout");
      changeFSMExecState(GEN_NEW_TRAJ, "MAP_STALE");
    }
    return;
  }

  if (have_target_ && planner_manager_->edt_environment_->sdf_map_->hasReadyMap()) {
    auto edt_env = planner_manager_->edt_environment_;

    double dist = planner_manager_->pp_.dynamic_ ?
        edt_env->evaluateCoarseEDT(end_pt_, /* time to program start + */ info->duration_) :
        edt_env->evaluateCoarseEDT(end_pt_, -1.0);

    if (dist <= 0.3) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Rejecting goal [%.3f, %.3f, %.3f]: obstacle clearance %.3f m is below 0.300 m",
                   end_pt_(0), end_pt_(1), end_pt_(2), dist);

      // Stop the active command at its current point. Do not replace the
      // requested goal with a nearby point and later report that as success.
      emergencyStop("goal is inside obstacle clearance");
      have_target_ = false;
      changeFSMExecState(WAIT_TARGET, "GOAL_IN_COLLISION");
      visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
      return;
    }
  }

  /* ---------- check trajectory ---------- */
  if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ) {
    double dist;
    bool   safe = planner_manager_->checkTrajCollision(dist);

    if (!safe) {
      // cout << "current traj in collision." << endl;
      RCLCPP_ERROR(node_->get_logger(),
                   "Active trajectory collides in %.2f m; holding position and replanning", dist);
      emergencyStop("collision on active trajectory");
      changeFSMExecState(GEN_NEW_TRAJ, "SAFETY");
    }
  }
}

bool KinoReplanFSM::callKinodynamicReplan() {
  auto plan_success = planner_manager_->kinodynamicReplan(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_);
  if (plan_success) {

    planner_manager_->planYaw(start_yaw_);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    quadrotor_msgs::msg::Bspline bspline;
    bspline.order      = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id    = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();

    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }

    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getInterval();

    bspline_pub_->publish(bspline);

    /* visulization */
    auto plan_data = &planner_manager_->plan_data_;
    visualization_->drawGeometricPath(plan_data->kino_path_, 0.075, Eigen::Vector4d(1, 1, 0, 0.4));
    visualization_->drawBspline(info->position_traj_, 0.1, Eigen::Vector4d(1.0,0, 0.0, 1), true, 0.2, Eigen::Vector4d(1, 0, 0, 1));

    return true;

  } else {
    cout << "generate new traj fail." << endl;
    return false;
  }
}

}  // namespace fast_planner
