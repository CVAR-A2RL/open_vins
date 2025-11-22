# Threading Issue with Pose Integration

## Problem

The external pose integration is causing segfaults **after successful updates**. The updates pass all tests (chi-squared, residual checks, covariance validation), but the system crashes shortly after.

## Root Cause

**Thread Safety Issue**: The pose callback is directly calling `try_update()` which modifies the EKF state immediately. Meanwhile, other threads (IMU propagation, camera tracking, MSCKF updates) may be reading or modifying the same state, causing race conditions.

### Evidence:
```
[POSE]: Successful update at time X.XXX (chi2 = 0.83 < 12.59)
[POSE]: Update successful at timestamp X.XXX
[ERROR]: process has died [exit code -11]  <-- Crash happens AFTER update completes
```

## Why This Happens

1. **ROS callback thread** receives pose message
2. **Callback directly calls** `VioManager::feed_measurement_pose()`
3. **This immediately calls** `updaterPOSE->try_update(state, ...)`
4. **State is modified** (position, orientation, covariance) without synchronization
5. **Main tracking thread** or another callback tries to access state
6. **Race condition** → segfault

### Comparison with Other Sensors

- **IMU**: Just queues data, doesn't update state directly
- **Camera**: Queues images, main thread processes them
- **Pose**: Currently updates state immediately (❌ unsafe)

## Temporary Workarounds

### Option 1: Disable Pose Updates (Safest)
```yaml
# In your estimator_config.yaml:
use_pose_updates: false
```

This keeps your system stable while a proper fix is developed.

---

### Option 2: Reduce Update Rate
Publish pose measurements at a **much lower rate** (e.g., 1-2 Hz instead of 10-30 Hz). This reduces the chance of collisions.

```python
# In your pose publisher:
self.timer = self.create_timer(1.0, self.publish_pose)  # 1 Hz instead of 10 Hz
```

---

### Option 3: Increase Measurement Covariance
Make the covariance **much larger** to reduce the magnitude of state changes:

```python
# Make updates very conservative:
cov[0] = cov[7] = cov[14] = 1.0       # 1m std for position (was 0.1)
cov[21] = cov[28] = cov[35] = 0.1     # ~5.7 deg std for orientation (was 0.01)
```

This makes each update smaller, reducing the impact of race conditions (but doesn't eliminate them).

---

### Option 4: Only Update After Convergence
Delay pose updates until VIO has been running for a while:

```python
class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.start_time = self.get_clock().now()
        self.warmup_duration = 10.0  # Wait 10 seconds before sending poses
        
    def publish_pose(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < self.warmup_duration:
            return  # Skip publishing during warmup
        
        # Publish pose...
```

---

## Proper Solution (Requires Code Changes)

The correct fix requires **restructuring the pose update to be thread-safe**:

### Architecture Change Needed:

```cpp
// Current (UNSAFE):
void VioManager::feed_measurement_pose(const PoseData &message) {
    updaterPOSE->feed_pose(message, oldest_time);
    bool did_update = updaterPOSE->try_update(state, message.timestamp);  // ❌ Direct state modification
}

// Proper (SAFE):
void VioManager::feed_measurement_pose(const PoseData &message) {
    // Just queue the measurement, don't update immediately
    updaterPOSE->feed_pose(message, oldest_time);  // ✓ Only queue
}

// Then in the main thread loop:
void VioManager::do_feature_propagate_update(...) {
    // ... existing camera processing ...
    
    // Try pose updates from queue (synchronized with other updates)
    if (updaterPOSE != nullptr) {
        updaterPOSE->try_queued_updates(state);  // ✓ Process in main thread
    }
}
```

### Required Changes:

1. **Remove direct `try_update()` call** from `feed_measurement_pose()`
2. **Add pose update processing** to main tracking thread
3. **Ensure proper locking** around state modifications
4. **Handle timing correctly** to apply updates at the right timestamps

---

## Current Status

**The pose integration works correctly** - updates pass all validation checks. The issue is **architectural** (threading), not mathematical.

### What's Working:
- ✅ Pose measurements are received correctly
- ✅ Residuals are reasonable (<1m position, <1 deg orientation)
- ✅ Chi-squared test passes
- ✅ EKF update math is correct
- ✅ Covariance remains positive definite

### What's Not Working:
- ❌ Thread safety causes race conditions
- ❌ System crashes after successful updates
- ❌ Cannot run reliably with high-rate pose inputs

---

## Diagnostic Commands

Check if threading is the issue:

```bash
# Run with low pose update rate:
ros2 topic hz /external_pose
# If you see crashes at ~1 Hz, threading is likely the issue

# Check if other updates are happening simultaneously:
ros2 topic hz /camera_topic
ros2 topic hz /imu_topic
# If these are high rate (>30 Hz) during crashes, confirms threading issue
```

---

## Recommendations

### For Testing/Development:
Use **Option 2** (low rate) or **Option 3** (high covariance)

### For Production:
Implement the **Proper Solution** with thread-safe architecture

### Quick Fix:
Use **Option 1** (disable) until proper threading is implemented

---

## Next Steps

1. **Confirm threading is the issue** by testing with 1 Hz pose rate
2. **Implement proper queueing** similar to IMU/camera measurements
3. **Add mutex protection** around state modifications
4. **Test with high-rate inputs** after fix

---

## Technical Notes

### Why IMU Doesn't Have This Issue:
- IMU data is queued in `propagator->feed_imu()`
- Propagation happens in `do_feature_propagate_update()` (main thread)
- Synchronized with camera processing

### Why We Can't Just Add a Mutex:
- Simple mutex would block the callback thread
- Could cause callback queue to overflow
- Could introduce deadlocks with nested locks
- Proper solution is to decouple measurement ingestion from state updates

---

## Example: Safe Low-Rate Test

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class SafePosePublisher(Node):
    def __init__(self):
        super().__init__('safe_pose_publisher')
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/external_pose',
            10
        )
        # Low rate: 1 Hz
        self.timer = self.create_timer(1.0, self.publish_pose)
        self.counter = 0
        self.get_logger().info('Safe pose publisher (1 Hz) started')
        
    def publish_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        
        # Your pose data...
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        
        # Large conservative covariance (1m position, ~5.7 deg orientation)
        cov = [0.0] * 36
        cov[0] = cov[7] = cov[14] = 1.0       # 1m std
        cov[21] = cov[28] = cov[35] = 0.1     # ~5.7 deg std
        msg.pose.covariance = cov
        
        self.publisher.publish(msg)
        self.counter += 1
        
        if self.counter % 10 == 0:
            self.get_logger().info(f'Published {self.counter} poses (1 Hz, safe rate)')

def main():
    rclpy.init()
    node = SafePosePublisher()
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

Test with this and see if crashes still occur. If they stop, threading is confirmed as the issue.

---

## Summary

**The pose integration code is correct**, but needs architectural changes for thread safety. Use the workarounds above while developing a proper solution, or disable pose updates if reliability is critical.
