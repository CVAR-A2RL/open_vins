# Testing External Pose Integration

This guide shows you how to verify that the external pose integration is working correctly.

## 1. Check That OpenVINS Started Successfully

When you launch OpenVINS, you should see these messages without crashes:

```bash
VioManager.cpp:169 UpdaterPose initialized for external pose measurements
ROS2Visualizer.cpp:50 Publishing: /ov_msckf/poseimu
ROS2Visualizer.cpp:52 Publishing: /ov_msckf/odomimu
ROS2Visualizer.cpp:54 Publishing: /ov_msckf/pathimu
...
subscribing to IMU: /your_imu_topic
subscribing to POSE: /external_pose     <-- ✓ Look for this line!
subscribing to cam (mono): /your_camera_topic
```

**✓ If you see "subscribing to POSE"** - The subscriber is active!

**✗ If you DON'T see it** - Check that `use_pose_updates: true` in your config file.

---

## 2. Verify the Pose Topic Subscription

### Check if OpenVINS is subscribed to your pose topic:

```bash
ros2 topic info /external_pose
```

**Expected output:**
```
Type: geometry_msgs/msg/PoseWithCovarianceStamped
Publisher count: X
Subscription count: 1   <-- ✓ OpenVINS should be subscribed
```

### List all subscribers:
```bash
ros2 topic info /external_pose -v
```

You should see `/ov_msckf` in the subscribers list.

---

## 3. Monitor Incoming Pose Messages

### Check if pose messages are being published:

```bash
ros2 topic hz /external_pose
```

**Expected output:**
```
average rate: 10.000   <-- Should match your publisher rate
```

### See the actual pose data:

```bash
ros2 topic echo /external_pose --once
```

**Expected output:**
```yaml
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: 'world'
pose:
  pose:
    position:
      x: 1.234
      y: 2.345
      z: 0.567
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  covariance: [0.01, 0.0, 0.0, ...]  <-- 36 elements
```

---

## 4. Check OpenVINS Debug Output

### Look for pose update messages in the terminal:

**When pose is received:**
```
[ROS-POSE]: Received pose at timestamp 1234567890.123
```

**When update is successful:**
```
[POSE]: Successful update at time 1234567890.123 (chi2 = 2.34 < 12.59)
[POSE]: Position residual: [0.012, -0.034, 0.005]
[POSE]: Orientation residual: [0.123, -0.456, 0.789] deg
```

**If update is rejected:**
```
[POSE]: Chi-squared test failed (25.34 > 12.59), rejecting measurement
```

### Increase verbosity to see more details:

Launch with `verbosity:=DEBUG` to see all pose-related messages.

---

## 5. Create a Test Pose Publisher

If you don't have a pose source yet, create a simple test publisher:

### Python Test Publisher:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

class TestPosePublisher(Node):
    def __init__(self):
        super().__init__('test_pose_publisher')
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/external_pose',
            10
        )
        self.timer = self.create_timer(0.1, self.publish_pose)  # 10 Hz
        self.counter = 0
        self.get_logger().info('Test pose publisher started')
        
    def publish_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        
        # Create a circular motion for testing
        t = self.counter * 0.1
        radius = 2.0
        msg.pose.pose.position.x = radius * math.cos(t)
        msg.pose.pose.position.y = radius * math.sin(t)
        msg.pose.pose.position.z = 1.0
        
        # Identity orientation (no rotation)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        
        # Conservative covariance (10cm position, 5deg orientation)
        cov = [0.0] * 36
        cov[0] = cov[7] = cov[14] = 0.01      # position variance (0.1m std)
        cov[21] = cov[28] = cov[35] = 0.01    # orientation variance (~5.7 deg std)
        msg.pose.covariance = cov
        
        self.publisher.publish(msg)
        self.counter += 1
        
        if self.counter % 10 == 0:
            self.get_logger().info(f'Published {self.counter} poses')

def main():
    rclpy.init()
    node = TestPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Save as:** `test_pose_publisher.py`

**Run it:**
```bash
chmod +x test_pose_publisher.py
python3 test_pose_publisher.py
```

---

## 6. Visualize in RViz2

### Launch RViz2:
```bash
ros2 run rviz2 rviz2
```

### Add these displays:

1. **OpenVINS Path** (VIO estimate):
   - Add → By topic → `/ov_msckf/pathimu` → Path

2. **OpenVINS Pose** (VIO with pose updates):
   - Add → By topic → `/ov_msckf/poseimu` → PoseWithCovariance

3. **External Pose** (your pose measurements):
   - Add → By topic → `/external_pose` → PoseWithCovariance

4. **Set Fixed Frame:** `world` or `global`

**What to look for:**
- Both paths should be visible
- If pose updates are working, the VIO path should be influenced by external poses
- Covariance ellipsoids show uncertainty

---

## 7. Compare VIO Output With and Without Pose Updates

### Test A: Run WITHOUT pose updates
```yaml
# In config file:
use_pose_updates: false
```

Launch and record the path:
```bash
ros2 bag record /ov_msckf/pathimu -o test_without_pose
```

### Test B: Run WITH pose updates
```yaml
# In config file:
use_pose_updates: true
```

Launch, publish test poses, and record:
```bash
ros2 bag record /ov_msckf/pathimu -o test_with_pose
```

### Compare the results:
The path with pose updates should show:
- Reduced drift (if your poses are accurate)
- Smoother trajectory (if pose rate is good)
- Different global position alignment

---

## 8. Monitor Performance

### Check CPU usage:
```bash
top -p $(pgrep -f run_subscribe_msckf)
```

Pose updates should add minimal overhead (<1% CPU typically).

### Check update rate:
```bash
# In OpenVINS terminal, count successful updates
grep -c "POSE.*Successful update" output.log
```

---

## 9. Common Issues and Solutions

### Issue: No "subscribing to POSE" message

**Cause:** `use_pose_updates` not enabled

**Solution:**
```yaml
# Add to your config file:
use_pose_updates: true
```

---

### Issue: "Received pose" but no "Successful update"

**Cause:** Updates being rejected (chi-squared test failure)

**Check:**
1. Is VIO initialized? (pose updates only work after initialization)
2. Are frames aligned? (global frame must match)
3. Is covariance too small? (increase by 10x-100x)

**Solution:**
```python
# Make covariance more conservative:
cov[0] = cov[7] = cov[14] = 0.1        # 0.1 → 0.1m std (was too small)
cov[21] = cov[28] = cov[35] = 0.1      # 0.01 → 0.1 rad std (more conservative)
```

---

### Issue: All updates rejected with high chi-squared values

**Cause:** Frame mismatch or incorrect covariance

**Debug steps:**
```bash
# 1. Check the residuals in debug output
# Look for: [POSE]: Position residual: [x, y, z]
# Large values (>1.0m) indicate frame mismatch

# 2. Check if timestamps are synchronized
ros2 topic echo /external_pose --field header.stamp
ros2 topic echo /imu_topic --field header.stamp

# 3. Verify your pose is in the correct frame
# The pose should be: position in global frame, orientation from global to IMU
```

---

### Issue: VIO diverges after pose updates start

**Cause:** Incorrect frame convention or bad covariance

**Solution:**
1. Verify your pose orientation convention (quaternion from global to IMU)
2. Make covariance MUCH larger initially (10x-100x)
3. Check if your pose source has outliers

---

## 10. Success Indicators

### ✓ System is working correctly if you see:

1. **Startup:**
   - "UpdaterPose initialized"
   - "subscribing to POSE: /your_topic"

2. **Runtime:**
   - "Received pose" messages at your publish rate
   - "Successful update" messages (>80% acceptance rate is good)
   - Chi-squared values well below threshold

3. **Behavior:**
   - VIO estimate converges toward pose measurements
   - No crashes or segfaults
   - Reasonable CPU usage

### Example of good output:
```
[POSE]: Received pose at timestamp 1000.100
[POSE]: Successful update at time 1000.100 (chi2 = 3.45 < 12.59)
[POSE]: Position residual: [0.012, -0.023, 0.005]
[POSE]: Orientation residual: [0.234, -0.123, 0.456] deg
[POSE]: Received pose at timestamp 1000.200
[POSE]: Successful update at time 1000.200 (chi2 = 2.89 < 12.59)
...
```

---

## 11. Advanced Debugging

### Enable detailed logging:

Add to your launch:
```bash
verbosity:=DEBUG
```

Or set ROS logging level:
```bash
ros2 run ov_msckf run_subscribe_msckf --ros-args --log-level debug
```

### Check parameter values:
```bash
ros2 param list /ov_msckf
ros2 param get /ov_msckf use_pose_updates
ros2 param get /ov_msckf topic_pose
```

### Monitor all OpenVINS topics:
```bash
ros2 topic list | grep ov_msckf
```

---

## 12. Quick Verification Checklist

- [ ] `use_pose_updates: true` in config file
- [ ] OpenVINS starts without crashes
- [ ] "subscribing to POSE" appears in output
- [ ] Pose messages are being published (`ros2 topic hz /external_pose`)
- [ ] OpenVINS is subscribed (`ros2 topic info /external_pose`)
- [ ] VIO initializes successfully
- [ ] "Received pose" messages appear
- [ ] "Successful update" messages appear (not all rejected)
- [ ] Chi-squared values are reasonable (<threshold)
- [ ] No segfaults or crashes during operation

---

## Need Help?

If you're still having issues:

1. Check the log files in OpenVINS output
2. Verify frame conventions match your system
3. Try the test publisher to isolate issues
4. Increase covariance if updates are rejected
5. Check the detailed guides:
   - `POSE_INTEGRATION_GUIDE.md` - Technical details
   - `QUICK_START_POSE_INTEGRATION.md` - Quick reference

Good luck! 🚀
