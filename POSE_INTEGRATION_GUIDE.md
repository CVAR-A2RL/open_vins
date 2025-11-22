# External Pose with Covariance Integration Guide

This document describes the changes made to integrate external pose measurements with covariance into OpenVINS MSCKF.

## Overview

The implementation adds support for fusing external 6DOF pose measurements (position + orientation with covariance) from sources like motion capture systems, GPS+compass, or other localization systems into the VIO estimate.

## Files Modified/Created

### 1. Core Data Structure
- **Modified**: `ov_core/src/utils/sensor_data.h`
  - Added `PoseData` struct containing timestamp, position, orientation (quaternion), and 6x6 covariance matrix

### 2. Updater Implementation
- **Created**: `ov_msckf/src/update/UpdaterPose.h`
  - Header file defining the `UpdaterPose` class
  
- **Created**: `ov_msckf/src/update/UpdaterPose.cpp`
  - Implementation of EKF update using pose measurements
  - Includes chi-squared test for outlier rejection
  - Handles position and orientation measurements with proper Jacobians

### 3. VIO Manager Integration
- **Modified**: `ov_msckf/src/core/VioManager.h`
  - Added forward declaration for `UpdaterPose`
  - Added `feed_measurement_pose()` function declaration
  - Added `updaterPOSE` member variable

- **Modified**: `ov_msckf/src/core/VioManager.cpp`
  - Added include for `UpdaterPose.h`
  - Implemented `feed_measurement_pose()` function
  - Initialize `updaterPOSE` in constructor when enabled

### 4. Configuration Options
- **Modified**: `ov_msckf/src/core/VioManagerOptions.h`
  - Added `use_pose_updates` boolean parameter
  - Added configuration parsing and printing for the new parameter

### 5. ROS2 Integration
- **Modified**: `ov_msckf/src/ros/ROS2Visualizer.h`
  - Added `callback_pose()` function declaration
  - Added `sub_pose` subscriber member variable

- **Modified**: `ov_msckf/src/ros/ROS2Visualizer.cpp`
  - Implemented `callback_pose()` to convert ROS2 messages to OpenVINS format
  - Added pose topic subscription in `setup_subscribers()`
  - Converts `PoseWithCovarianceStamped` messages to internal format

### 6. Build System
- **Modified**: `ov_msckf/cmake/ROS2.cmake`
  - Added `src/update/UpdaterPose.cpp` to `LIBRARY_SOURCES`

- **Modified**: `ov_msckf/cmake/ROS1.cmake`
  - Added `src/update/UpdaterPose.cpp` to library sources (for ROS1 compatibility)

## Configuration

To enable external pose updates, add the following to your configuration YAML file:

```yaml
# Enable external pose updates
use_pose_updates: true

# Optional: specify the topic name (default: /external_pose)
# This can also be set as a ROS parameter
```

You can also override the topic at runtime:
```bash
ros2 run ov_msckf run_subscribe_msckf --ros-args -p topic_pose:=/your_pose_topic
```

## Technical Details

### Measurement Model
The updater implements the following measurement model:
- **Position**: `z_p = p_IinG + n_p`
- **Orientation**: `z_q = q_GtoI ⊗ exp(n_q)`

where `n_p` and `n_q` are zero-mean Gaussian noise terms.

### State Update
The EKF update affects the IMU pose state variables:
- Orientation quaternion `q_GtoI` (3 DOF after manifold parameterization)
- Position `p_IinG` (3 DOF)

### Features
1. **Chi-squared test**: Outlier rejection using 95% confidence threshold
2. **Timestamp matching**: Finds closest measurement within 0.1 seconds
3. **Measurement buffering**: Maintains a queue of pose measurements
4. **Automatic cleanup**: Removes old measurements based on state margin time

### Message Format
The system expects `geometry_msgs/PoseWithCovarianceStamped` messages:
- Position: meters in global frame
- Orientation: quaternion (x, y, z, w)
- Covariance: 6x6 matrix [position; orientation] in row-major order

## Building

After making these changes, rebuild the workspace:

```bash
cd /root/as2_thirdparties_ws
colcon build --packages-select ov_core ov_msckf
source install/setup.bash
```

## Usage Example

1. **Set configuration**:
   ```yaml
   # In your config file (e.g., estimator_config.yaml)
   use_pose_updates: true
   ```

2. **Launch OpenVINS**:

   **Option A: Direct execution**
   ```bash
   ros2 run ov_msckf run_subscribe_msckf
   ```

   **Option B: Using launch file**
   ```bash
   ros2 launch ov_msckf subscribe.launch.py \
       config:=your_config \
       config_path:=config/your_config/estimator_config.yaml \
       verbosity:=DEBUG \
       use_stereo:=false \
       max_cameras:=1 \
       use_sim_time:=true \
       topic_pose:=/your_pose_topic
   ```

   Or with your specific syntax:
   ```bash
   ros2 launch ov_msckf subscribe.launch.py \
       config:=<%= nx_user %> \
       config_path:=config/<%= nx_user %>/open_vins/estimator_config.yaml \
       verbosity:=DEBUG \
       use_stereo:=false \
       max_cameras:=1 \
       use_sim_time:=true \
       topic_pose:=/external_pose
   ```

3. **Publish pose measurements**:
   Your external system should publish to the configured topic (default: `/external_pose`):
   ```bash
   ros2 topic echo /external_pose geometry_msgs/msg/PoseWithCovarianceStamped
   ```

## Debugging

Check if pose measurements are being received:
```bash
# In OpenVINS output, look for:
[POSE]: Successful update at time X.XXX (chi2 = Y.YY < Z.ZZ)
[POSE]: Position residual: [x, y, z]
[POSE]: Orientation residual: [rx, ry, rz] deg
```

If updates are rejected:
```bash
[POSE]: Chi-squared test failed (X.XX > Y.YY), rejecting measurement
```

This indicates the measurement is inconsistent with the current state estimate. Check:
- Covariance values (may be too small/optimistic)
- Frame conventions (ensure global frame alignment)
- Time synchronization

## Notes

1. **Frame Convention**: Ensure your pose is in the same global frame as VIO
2. **Quaternion Convention**: OpenVINS uses JPL quaternion convention (Hamilton is also supported)
3. **Time Synchronization**: Pose timestamps must be synchronized with IMU/camera timestamps
4. **Initialization**: The VIO system must be initialized before pose updates are processed
5. **Covariance Tuning**: You may need to scale the covariance from your sensor to achieve optimal fusion

## Future Enhancements

Possible improvements:
- Add support for pose-only (no covariance) measurements
- Implement temporal interpolation for better timestamp alignment
- Add configuration for measurement frequency limits
- Support for different reference frames
