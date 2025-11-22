# Quick Start: External Pose Integration

This is a quick reference for integrating external pose measurements into your OpenVINS setup.

## Prerequisites

✅ All code changes have been applied  
✅ Build the workspace: `colcon build --packages-select ov_core ov_msckf`  
✅ Source the workspace: `source install/setup.bash`

## Step 1: Enable in Your Config

Edit your estimator config file at:
`config/<%= nx_user %>/open_vins/estimator_config.yaml`

Add this line:
```yaml
# Enable external pose updates
use_pose_updates: true
```

## Step 2: Launch OpenVINS

With your existing launch command, just add the `topic_pose` parameter:

```bash
ros2 launch ov_msckf subscribe.launch.py \
    config:=<%= nx_user %> \
    config_path:=config/<%= nx_user %>/open_vins/estimator_config.yaml \
    verbosity:=DEBUG \
    use_stereo:=false \
    max_cameras:=1 \
    use_sim_time:=true \
    topic_pose:=/your_pose_topic_name
```

**Default topic**: If you don't specify `topic_pose`, it defaults to `/external_pose`

## Step 3: Verify It's Working

### Check if the subscriber is active:
```bash
ros2 topic info /external_pose  # or your custom topic name
```

You should see OpenVINS as a subscriber.

### Look for debug messages in OpenVINS output:
```
subscribing to POSE: /external_pose
[POSE]: Received pose at timestamp X.XXX
[POSE]: Successful update at time X.XXX (chi2 = Y.YY < Z.ZZ)
```

## Message Format

Your pose publisher must send `geometry_msgs/msg/PoseWithCovarianceStamped`:

```python
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

msg = PoseWithCovarianceStamped()
msg.header.stamp = self.get_clock().now().to_msg()
msg.header.frame_id = "global"  # or your global frame

# Position (meters)
msg.pose.pose.position.x = x
msg.pose.pose.position.y = y
msg.pose.pose.position.z = z

# Orientation (quaternion: x, y, z, w)
msg.pose.pose.orientation.x = qx
msg.pose.pose.orientation.y = qy
msg.pose.pose.orientation.z = qz
msg.pose.pose.orientation.w = qw

# Covariance (6x6 matrix: [position; orientation])
# Order: x, y, z, roll, pitch, yaw
# Row-major format (36 elements)
msg.pose.covariance = [
    0.01, 0.0,  0.0,  0.0, 0.0, 0.0,  # x variance and covariances
    0.0,  0.01, 0.0,  0.0, 0.0, 0.0,  # y variance and covariances
    0.0,  0.0,  0.01, 0.0, 0.0, 0.0,  # z variance and covariances
    0.0,  0.0,  0.0,  0.1, 0.0, 0.0,  # roll variance and covariances
    0.0,  0.0,  0.0,  0.0, 0.1, 0.0,  # pitch variance and covariances
    0.0,  0.0,  0.0,  0.0, 0.0, 0.1   # yaw variance and covariances
]

publisher.publish(msg)
```

## Troubleshooting

### No updates happening?

1. **Check VIO initialization**: Pose updates only work after VIO is initialized
   ```
   # Look for this in OpenVINS output:
   [INIT]: orientation = ...
   [INIT]: ...
   ```

2. **Check timing**: Pose timestamps must be within 0.1 seconds of state time
   ```bash
   ros2 topic echo /external_pose --field header.stamp
   ```

3. **Verify use_pose_updates is true**:
   ```bash
   ros2 param get /ov_msckf/run_subscribe_msckf use_pose_updates
   ```

### Updates being rejected?

If you see:
```
[POSE]: Chi-squared test failed (X.XX > Y.YY), rejecting measurement
```

This means the measurement is inconsistent with VIO estimate. Try:
1. **Increase covariance values** (make them less optimistic)
2. **Check frame alignment** (ensure global frames match)
3. **Verify time sync** (ensure clocks are synchronized)

### Common Issues

| Issue | Solution |
|-------|----------|
| "No ROS parameter topic_pose found" | Add `topic_pose:=...` to launch command |
| No pose subscriber created | Check `use_pose_updates: true` in config |
| All measurements rejected | Increase covariance by 10x-100x |
| VIO diverges after pose update | Covariance too small or frame mismatch |

## Frame Conventions

⚠️ **IMPORTANT**: Your pose must be in the correct frame!

- **Position**: In the same global/world frame as VIO (typically ENU or NED)
- **Orientation**: Quaternion from global frame to IMU frame
- **Units**: Meters and radians

If your frames don't match, transform the pose before publishing.

## Performance Tips

1. **Publish rate**: 10-30 Hz is usually sufficient (don't need to match IMU rate)
2. **Covariance tuning**: Start conservative (large values), then decrease
3. **Chi-squared multiplier**: Can be adjusted in config if needed
4. **Measurement delay**: System handles up to ~100ms delay automatically

## Example Publisher (Python)

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/external_pose', 
            10
        )
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        
    def timer_callback(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        
        # Get your pose data here
        x, y, z, qx, qy, qz, qw = self.get_pose()
        
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        
        # Covariance (conservative values)
        pos_var = 0.1  # 10cm std dev
        ori_var = 0.01  # ~5.7 deg std dev
        cov = [0.0] * 36
        cov[0] = cov[7] = cov[14] = pos_var  # position
        cov[21] = cov[28] = cov[35] = ori_var  # orientation
        msg.pose.covariance = cov
        
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = PosePublisher()
    rclpy.spin(node)
    
if __name__ == '__main__':
    main()
```

## Next Steps

For detailed information, see `POSE_INTEGRATION_GUIDE.md`
