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

Build the planner after building `plan_ws` (which provides the shared
`quadrotor_msgs` package):

```bash
source /opt/ros/jazzy/setup.bash
source /home/thunder/GRA_1/plan_ws/install/setup.bash
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

Do not run EGO-Planner at the same time. Both planners publish flight
commands and the bridge accepts only one active command stream.
