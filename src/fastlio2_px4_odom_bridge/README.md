# FAST-LIO2 to PX4 EKF bridge

This package converts `/fastlio2/lio_odom` (`nav_msgs/msg/Odometry`) into
`/fmu/in/vehicle_visual_odometry` (`px4_msgs/msg/VehicleOdometry`). It only
publishes an external-odometry input. It does not publish Offboard setpoints
or change PX4 EKF parameters.

Build and run after sourcing the `px4_msgs` overlay:

```bash
cd /home/thunder/GRA_1/Fast-Planner-ROS2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select fastlio2_px4_odom_bridge
source install/setup.bash
ros2 launch fastlio2_px4_odom_bridge fastlio2_px4_odom_bridge.launch.py
```

The existing PX4 `bringup.launch.py` starts this node by default alongside
`px4_bridge`. Disable it with `start_odom_bridge:=false`, or override its
configuration with `odom_bridge_params:=/path/to/file.yaml`.

PX4 parameter changes are separate from starting the ROS node. Apply and save
the EKF2 settings once in the PX4 shell (or QGroundControl); PX4 SITL restores
saved parameters on subsequent launches:

```text
param set EKF2_EV_CTRL 3
param set EKF2_EV_DELAY 0
param set EKF2_EV_NOISE_MD 0
param save
```

`EKF2_EV_CTRL=3` enables horizontal and vertical position fusion in current
PX4. Add velocity (`7`) and yaw (`15`) only after validating the corresponding
signals. Parameter meanings can vary by PX4 release; check the parameter
metadata for the exact firmware revision. The current `bringup` launch does
not inject parameter commands into PX4 at startup.

Important assumptions and follow-up work:

* The default world rotation matches the existing planner bridge:
  `PX4_NED = [lidar_y, lidar_x, -lidar_z]`. Verify this with a small-axis test
  before enabling EKF fusion.
* In this Gazebo setup the MID-360 plugin rotates its IMU output back from the
  sensor's pi-yaw mounting so `/livox/imu` and FAST-LIO `body` axes align with
  X500 `base_link` (FLU: forward, left, up). PX4 uses FRD (forward, right,
  down), so `body_to_frd_row_major` is `diag(1,-1,-1)`, a 180 degree rotation
  about X. Real hardware must use its measured IMU/body-to-FRD rotation.
* The node rejects an odometry message unless `header.frame_id` is `lidar`.
  It does not use TF to silently transform data.
* The default timestamp mode uses the ROS clock at publish time. Use
  `odom_stamp` only after confirming that the ROS and PX4 time bases are
  aligned; otherwise EKF2 will reject or mis-age measurements.
* `transform_translation` is zero by default. This assumes the FAST-LIO local
  origin and PX4 local NED origin have been aligned. A nonzero calibrated
  translation is required when they are not aligned.
* Configure PX4 EKF2 external-vision fusion separately (`EKF2_EV_CTRL`, delay,
  sensor position, and height reference as appropriate for the PX4 version).
  Check `EstimatorStatus` flags `CS_EV_POS`, `CS_EV_VEL`, `CS_EV_HGT`, and
  `CS_EV_YAW`, plus EV innovations, to prove that fusion is active.
* Do not run this node and another visual-odometry publisher simultaneously.
  Duplicate publishers can make the EKF input intermittent.
