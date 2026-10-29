# Fast-Planner integration

The package boundaries follow the existing EGO workspace: ROS wiring lives in
the Python launch file, planner tuning lives in
`src/fast_planner/plan_manage/config/fast_planner_lidar.yaml`, and the
trajectory optimizer owns its solver dependency.

This workspace is configured for the existing Jazzy simulation stack:

- odometry: `/fastlio2/lio_odom`
- world point cloud: `/fastlio2/world_cloud`
- goal input: `/goal_pose` (`geometry_msgs/msg/PoseStamped`, frame `lidar`)
- PX4 command output: `/drone_0_planning/pos_cmd`
- planner and bridge frame: `lidar`

Before flight, check `header.frame_id` on both `/fastlio2/lio_odom` and
`/fastlio2/world_cloud`. They must be the same frame and match
`planning_frame` (default `lidar`). The Fast-Planner map consumes cloud points
directly and refuses a cloud with a different frame; it does not apply TF.
If FAST-LIO reports another common frame, add a TF conversion before planning
and configure the bridge frame consistently; merely renaming the planner
frame does not transform the command coordinates. The current default matches
the PX4 bridge's `lidar` frame.

Build this workspace independently. It contains its own `quadrotor_msgs`,
`px4_msgs`, PX4 command bridge, and FAST-LIO2-to-PX4 odometry bridge:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/thunder/GRA_1/Fast-Planner-ROS2
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

The B-spline optimizer uses the same header-only L-BFGS implementation as EGO.
The existing Fast-Planner objective and gradients remain in place; its former
optimizer variable bounds are enforced through L-BFGS's step-bound callback.

Start the planner with:

```bash
ros2 launch plan_manage fast_planner_lidar.launch.py
```

Start the planner and its RViz view together with:

```bash
ros2 launch plan_manage fast_planner_lidar_rviz.launch.py
```

For the EGO-style SITL control path, start Fast-Planner, the PX4 Offboard
bridge, the FAST-LIO2-to-PX4 external-odometry fusion bridge, and RViz together
with:

```bash
source /home/thunder/GRA_1/Fast-Planner-ROS2/install/setup.bash
ros2 launch px4_bridge bringup.launch.py
```

Start the simulator and FAST-LIO first. This launch starts the Micro XRCE-DDS
agent by default; pass `start_agent:=false` if another agent already owns UDP
8888. The bringup starts `fastlio2_px4_odom_bridge` by default; pass
`start_odom_bridge:=false` only when the odometry fusion bridge is already
running. It keeps the EGO bridge contract: `/drone_0_planning/pos_cmd` in `lidar`,
transformed into PX4 NED, with Offboard position setpoints streamed at 20 Hz.
The SITL profile matches the EGO Gazebo run at 0.8 m/s planner velocity and
1.0 m/s^2 acceleration, with a 0.5 m startup takeoff and 5 m/s bridge
validation ceiling. The
planner only produces setpoints; it does not raise the PX4 speed ceiling.

The comparison baseline aligns Fast-Planner's map size, resolution, local
update range, obstacle inflation, ground/ceiling bounds, velocity and
acceleration limits with the current EGO launch. A larger map or different
inflation must be treated as a separate parameter experiment, because it
changes the planning problem rather than only the algorithm.

After startup, verify that `/px4_bridge/status` reaches
`offboard_active_vehicle_armed_tracking_latest_planner_setpoint` and inspect
`/drone_0_planning/pos_cmd` for nonzero velocity while moving. A visible path
alone only confirms planning and visualization, not that PX4 accepted the
setpoints. Keep EGO-Planner stopped while this launch controls the vehicle.

The RViz view includes the live world and inflated local point clouds, the
kinodynamic path and optimized B-spline, the commanded position, and the
measured odometry trail. `planning_frame` and `use_sim_time` are forwarded to
both planner processes and RViz. The map rejects mismatched frames, requires
recent point-cloud data before planning, and retries failed replans with a
short delay so an unavailable map does not trigger a tight planning loop.

FAST-LIO publishes pose and world points in `lidar`, while its odometry twist
is expressed in `child_frame_id` (`body`). Fast-Planner rotates that velocity
into `lidar` before using it as the kinodynamic initial state. The trajectory
server accepts the repeated endpoint knots used by clamped B-splines and
publishes position commands in `lidar`. Local replans start from measured
odometry and can be triggered by tracking error as well as trajectory progress.

Do not run EGO-Planner at the same time. Both planners publish flight
commands and the bridge accepts only one active command stream.
