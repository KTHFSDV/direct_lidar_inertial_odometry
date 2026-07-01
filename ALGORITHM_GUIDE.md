# DLIO (Direct Lidar-Inertial Odometry) - Technical Documentation

## Quick Reference

**Purpose**: Real-time localization combining LiDAR and IMU measurements  
**Language**: C++ (ROS2)  
**Update Rate**: 10-20 Hz (LiDAR scan rate)  
**Accuracy**: <1% of distance traveled  

---

## How It Works - Deep Dive

### 1. Point Cloud Deskewing

The key innovation of DLIO is **continuous-time motion correction** through two-stage deskewing.

#### Problem
Standard LiDAR scanners take ~100ms to capture all points in a single scan. During this time, if the vehicle is moving (especially aggressively in acceleration, braking, or turning), different points are captured from different positions/orientations. This causes the point cloud to appear "smeared" or distorted.

#### Solution: Coarse-to-Fine Deskewing

```
┌─────────────────────────────────────────┐
│  1. COARSE DESKEW (High-Freq IMU)       │
├─────────────────────────────────────────┤
│ For each point i in scan:               │
│   - Determine capture time: t_i         │
│   - Use IMU to estimate motion          │
│     between scan_start and t_i          │
│   - Transform point back to t_start     │
│                                         │
│ Result: Approximately deskewed points   │
└─────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────┐
│  2. FINE DESKEW (Point Cloud Reg.)      │
├─────────────────────────────────────────┤
│ 1. Register coarse-deskewed scan to     │
│    map using ICP                        │
│ 2. Measure registration error           │
│ 3. Use error to refine deskew estimate  │
│    (optimize deskew parameters)         │
│                                         │
│ Result: Highly accurate deskewed        │
│         point cloud                     │
└─────────────────────────────────────────┘
```

#### Mathematical Detail

For a point $p_i$ captured at time $t_i$ during scan interval $[t_0, t_1]$:

**Coarse deskewing**:
$$p'_i = T(t_0)^{-1} \cdot T_{IMU}(t_0 \to t_i) \cdot p_i$$

where:
- $T(t_0)$ = vehicle pose at scan start
- $T_{IMU}(t_0 \to t_i)$ = motion estimated from IMU data
- $p_i$ = original point in scanner frame

**Fine refinement**:
- Optimize deskew parameters to minimize ICP residual
- Adjust gyro/accel biases if needed
- Results in sub-centimeter accuracy

### 2. Scan-to-Map Registration (ICP)

DLIO uses **Iterative Closest Point (ICP)** to find the transformation between the current deskewed scan and the accumulated map.

#### Algorithm Steps

```
Iteration loop:
  1. Find nearest neighbors
     - For each point in current scan
     - Find closest point in map
     - Estimate plane normal at each match
     
  2. Solve point-to-plane registration
     minimize: Σ ||n_i^T (R*p_i + t - q_i)||²
     
     Variables: R (rotation matrix), t (translation vector)
     
     Expanded:
     - p_i = point from current scan
     - q_i = matched point in map
     - n_i = surface normal at match
     - Point-to-plane error more robust than point-to-point
     
  3. Apply transformation
     - Update rotation and translation estimate
     - Transform scan by new estimate
     
  4. Check convergence
     - If change < threshold: done
     - Otherwise: repeat with updated scan position
```

#### Why Point-to-Plane?

```
Point-to-Point:
  Error = ||R*p - q||  (treats as 3D distance)
  Problem: Outliers have large influence
  
Point-to-Plane:
  Error = ||n^T(R*p - q)||  (perpendicular distance to surface)
  Benefit: More robust to noise and outliers
  Geometric insight: We only care about fitting to surface, 
                    not exact point locations
```

### 3. Keyframe Management

DLIO maintains a sparse map using keyframes rather than storing every scan.

```
Keyframe Decision Logic:

┌─ New scan arrives
│
├─ Check distance threshold
│  If distance > dist_thresh: ADD KEYFRAME
│  Else: continue
│
├─ Check rotation threshold  
│  If rotation > rot_thresh: ADD KEYFRAME
│  Else: continue
│
└─ If thresholds not met: 
   Merge scan into current keyframe
   (don't add new keyframe)

Typical thresholds:
  Distance: 0.5-1.0 m
  Rotation: 5-10 degrees
```

**Benefits**:
- Maintains only ~10-100 keyframes instead of 1000+ scans
- Reduces map size from GB to MB
- Faster computation without loss of accuracy
- Can save map to disk for loop closure detection

### 4. IMU Integration (State Estimation)

DLIO maintains a Kalman Filter that fuses IMU and LiDAR measurements.

#### State Vector
```
State = [
  p_x, p_y, p_z,           # Position (m)
  v_x, v_y, v_z,           # Velocity (m/s)
  q_w, q_x, q_y, q_z,      # Orientation (quaternion)
  b_a_x, b_a_y, b_a_z,     # Acceleration bias
  b_w_x, b_w_y, b_w_z      # Gyro bias
]
```

#### Prediction Step (IMU at 100 Hz)

```
For each IMU measurement:

1. Correct IMU measurements
   accel_true = accel_raw - accel_bias
   gyro_true = gyro_raw - gyro_bias
   
2. Update orientation (quaternion integration)
   dq = 0.5 * q * [0; gyro_true] * dt
   q_new = q + dq
   
3. Remove gravity from acceleration
   accel_body = accel_true - R^T * g
   (where R is rotation matrix, g = [0,0,9.81])
   
4. Update velocity
   v_new = v + R * accel_body * dt
   
5. Update position  
   p_new = p + v * dt
   
6. Predict covariance
   P_new = A * P * A^T + Q
   (Accounts for increasing uncertainty)
```

#### Update Step (LiDAR at 10-20 Hz)

```
When new LiDAR scan provides measurement:

1. Get measurement from ICP registration
   z = [Δx, Δy, Δz, Δroll, Δpitch, Δyaw]
   R_meas = covariance of measurement
   
2. Compute innovation (residual)
   innovation = z - expected_measurement
   
3. Compute Kalman Gain
   K = P * H^T / (H * P * H^T + R_meas)
   (Balance between IMU prediction and LiDAR measurement)
   
4. Update state
   x_new = x + K * innovation
   
5. Update covariance
   P_new = (I - K*H) * P
   (Reduce uncertainty after measurement)
```

This elegant fusion gives:
- **High frequency**: IMU provides smooth updates at 100 Hz
- **Accuracy**: LiDAR corrects accumulating drift
- **Robustness**: Falls back to IMU if LiDAR fails

### 5. Configuration: Critical Parameters

```yaml
# MOTION CORRECTION
pointcloud/deskew: true      # Enable deskewing (crucial for accuracy)
pointcloud/voxelize: true    # Downsample (10-20% of points)

# IMU CALIBRATION
imu/calibration: true        # Auto-calibration (5 sec at startup)
imu/intrinsics/accel/bias: [0.26, 0.04, 0.13]  # m/s²
imu/intrinsics/accel/sm:     # Scale matrix (3x3 identity if ideal)
imu/intrinsics/gyro/bias: [0.019, -0.020, 0.003]  # rad/s

# EXTRINSICS (CRITICAL!)
# Position and rotation between sensors and vehicle center
extrinsics/baselink2lidar/t: [0.542, 0.0, 1.048]  # meters
extrinsics/baselink2imu/t:   [0.536, 0.012, 0.982]
# If these are wrong, results will be inaccurate!
```

**Extrinsics Meaning**:
- baselink = vehicle center (reference frame)
- If LiDAR is mounted 54cm forward and 105cm high
- Set extrinsics/baselink2lidar/t to [0.542, 0.0, 1.048]
- DLIO uses this to transform measurements to vehicle frame

---

## Algorithm Performance

### Computational Complexity

| Operation | Complexity | Time (for 100k points) |
|-----------|-----------|----------------------|
| Deskewing | O(n) | ~5-10 ms |
| ICP (5 iterations) | O(n log n) | ~20-30 ms |
| EKF update | O(n²) state | ~10-15 ms |
| **Total** | | ~50-100 ms |

### Memory Usage

- Current scan: ~5 MB (100k points)
- Submap (20-50 keyframes): ~50-250 MB
- IMU buffer (10 sec at 100 Hz): ~500 KB
- Total: ~50-300 MB (depending on map size)

### Accuracy Characteristics

```
Static positioning (no motion):
  Accuracy: <5 cm
  
Slow motion (walking pace):
  Drift: <0.5% of distance
  
Fast motion (vehicle speed):
  Drift: <1% of distance (with deskewing)
  Drift: 2-5% without deskewing
  
Sudden acceleration:
  Deskewing critical for accuracy
  Can maintain <2cm accuracy with good deskewing
  
Turn rate:
  Can handle up to 180°/sec with proper IMU calibration
```

---

## Common Issues and Solutions

### Issue: High Drift (Odometry drifts away from true path)

**Causes**:
1. IMU not calibrated → Solution: Enable imu/calibration, keep static for 10 sec
2. Extrinsics incorrect → Solution: Measure sensor positions carefully
3. LiDAR noisy → Solution: Increase voxel size or check sensor health

### Issue: Jerky/Inconsistent Output

**Causes**:
1. Deskewing disabled → Solution: Set pointcloud/deskew: true
2. IMU measurements unreliable → Solution: Check IMU cable/connection
3. Fast motion causing registration failure → Solution: Reduce deskew time window

### Issue: Complete Failure (odometry jumps/diverges)

**Causes**:
1. LiDAR and IMU not time-synchronized → Solution: Synchronize timestamps
2. Wrong message type → Solution: Check if using ros2can vs standard IMU
3. Bad initial alignment → Solution: Keep vehicle still at startup

---

## ROS Topics

### Input Subscriptions

```cpp
// LiDAR point cloud (standard)
/pointcloud : sensor_msgs/PointCloud2

// IMU (standard)
/imu : sensor_msgs/Imu

// OR IMU (ROS2CAN, for vehicle CAN integration)
/accel : ros2can_msgs/SbgEcanMsgImuAccel
/gyro : ros2can_msgs/SbgEcanMsgImuGyro  
/timestamp : ros2can_msgs/SbgEcanMsgImuInfo
```

### Output Publications

```cpp
// Main odometry output
/odom : nav_msgs/Odometry
  Contains: position, orientation, velocity, twist

// Convenience topic (just pose)
/pose : geometry_msgs/PoseStamped
  Contains: position and orientation only

// Complete trajectory
/path : nav_msgs/Path
  Contains: entire history of poses

// Keyframe poses
/kf_pose : geometry_msgs/PoseArray

// Deskewed point cloud (for visualization/debugging)
/deskewed : sensor_msgs/PointCloud2

// TF broadcast
/tf : geometry_msgs/TransformStamped
  Publishes transform from map → base_link
```

---

## Usage Example

```bash
ros2 launch direct_lidar_inertial_odometry dlio.launch.py

# View in RViz
rviz2

# In RViz, add topics:
# - /path (map frame) - trajectory visualization
# - /kf_cloud (map frame) - map point cloud
# - /deskewed (base_link frame) - debug scan
```

---

## Research Background

DLIO's continuous-time motion correction is based on the principle that:

1. **High-frequency IMU** provides smooth motion estimate
2. **Low-frequency LiDAR** provides accurate absolute correction
3. **Fusion** of both gives accurate AND smooth odometry

This is more sophisticated than simple odometry that:
- Either ignores motion during scan capture (causes blur)
- Or uses coarse constant-velocity assumption (assumes zero acceleration)

The paper "Direct LiDAR-Inertial Odometry: Lightweight LIO with Continuous-Time Motion Correction" (Chen et al., ICRA 2023) details this innovation.

---

## Performance Tips

1. **Extrinsic Calibration**: Most important for accuracy
   - Measure exactly where sensors are mounted
   - Use precision tools or markers
   
2. **IMU Mounting**: Keep away from vibration sources
   - Don't mount near motors
   - Use damping material
   
3. **LiDAR Mounting**: Needs clear field of view
   - Mount on top of vehicle
   - Avoid metallic reflectors nearby
   
4. **Time Synchronization**: Critical for multi-sensor fusion
   - Use hardware sync if available
   - Verify with sample rosbags

5. **Dynamic Parameter Tuning**:
   - Start with defaults
   - Increase voxel_size if CPU-constrained
   - Increase keyframe thresholds if map too large

---

**For EDE**: Be ready to explain:
- Why deskewing is necessary (aggressive acceleration in autocross)
- How IMU and LiDAR complement each other
- Why extrinsic calibration is critical
- Performance under different vehicle motions
