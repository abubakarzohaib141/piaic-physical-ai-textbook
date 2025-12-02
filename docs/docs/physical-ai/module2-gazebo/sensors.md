# Gazebo Sensors - Simulating Robot Perception

## Overview

For a humanoid robot to navigate and interact with the world, it needs **sensors**. Gazebo can simulate the three most critical sensor types:

1. **Cameras** (RGB, Depth, Stereo) - Vision
2. **LiDAR** - 3D environment mapping
3. **IMU** (Inertial Measurement Unit) - Balance and orientation

## Camera Sensors

### RGB Camera

Adding a camera to your URDF:

```xml
<!-- Camera link -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.02 0.05 0.02"/>
    </geometry>
    <material name="black"/>
  </visual>
  <collision>
    <geometry>
      <box size="0.02 0.05 0.02"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>

<!-- Camera joint (mounted on head) -->
<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.1 0 0" rpy="0 0 0"/>
</joint>

<!-- Gazebo camera plugin -->
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
    
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/humanoid</namespace>
        <argument>image_raw:=camera/image_raw</argument>
        <argument>camera_info:=camera/camera_info</argument>
      </ros>
      <camera_name>head_camera</camera_name>
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Subscribing to Camera Data

```python
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

class CameraProcessor(Node):
    def __init__(self):
        super().__init__('camera_processor')
        self.bridge = CvBridge()
        
        self.image_sub = self.create_subscription(
            Image,
            '/humanoid/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/humanoid/camera/camera_info',
            self.info_callback,
            10
        )
        
    def image_callback(self, msg):
        # Convert ROS Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Process image (object detection, etc.)
        processed = self.detect_objects(cv_image)
        
        # Display (optional)
        cv2.imshow('Camera View', processed)
        cv2.waitKey(1)
        
    def info_callback(self, msg):
        # Store camera calibration
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
```

### Depth Camera (Intel RealSense Simulation)

```xml
<gazebo reference="camera_link">
  <sensor name="depth_camera" type="depth">
    <update_rate>20</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
    </camera>
    
    <plugin name="depth_camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/humanoid</namespace>
      </ros>
      <camera_name>depth_camera</camera_name>
      <frame_name>camera_link</frame_name>
      <min_depth>0.1</min_depth>
      <max_depth>10.0</max_depth>
    </plugin>
  </sensor>
</gazebo>
```

This publishes:
- `/humanoid/depth_camera/image_raw` - RGB image
- `/humanoid/depth_camera/depth/image_raw` - Depth image
- `/humanoid/depth_camera/points` - Point cloud

## LiDAR Sensors

LiDAR provides 3D point clouds for navigation and obstacle avoidance.

### 2D LiDAR (Planar Scan)

```xml
<gazebo reference="lidar_link">
  <sensor name="lidar" type="ray">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.10</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </ray>
    
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/humanoid</namespace>
        <argument>~/out:=scan</argument>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Processing LiDAR Data

```python
from sensor_msgs.msg import LaserScan

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/humanoid/scan',
            self.scan_callback,
            10
        )
        
    def scan_callback(self, msg):
        # msg.ranges contains distance measurements
        # msg.angle_min, msg.angle_max, msg.angle_increment
        
        # Find closest obstacle
        min_distance = min(msg.ranges)
        min_index = msg.ranges.index(min_distance)
        angle = msg.angle_min + min_index * msg.angle_increment
        
        self.get_logger().info(
            f'Closest obstacle: {min_distance:.2f}m at {np.degrees(angle):.1f}°'
        )
        
        # Obstacle avoidance logic
        if min_distance < 0.5:
            self.get_logger().warn('OBSTACLE TOO CLOSE!')
```

### 3D LiDAR (Velodyne-style)

```xml
<gazebo reference="lidar_link">
  <sensor name="velodyne" type="gpu_ray">
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>1800</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
        <vertical>
          <samples>16</samples>
          <resolution>1</resolution>
          <min_angle>-0.2618</min_angle>  <!-- -15 degrees -->
          <max_angle>0.2618</max_angle>   <!-- +15 degrees -->
        </vertical>
      </scan>
      <range>
        <min>0.1</min>
        <max>100</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    
    <plugin name="gazebo_ros_velodyne_laser_controller" filename="libgazebo_ros_velodyne_laser.so">
      <ros>
        <namespace>/humanoid</namespace>
        <argument>~/out:=velodyne_points</argument>
      </ros>
      <frame_name>lidar_link</frame_name>
      <min_range>0.9</min_range>
      <max_range>100.0</max_range>
    </plugin>
  </sensor>
</gazebo>
```

## IMU (Inertial Measurement Unit)

Critical for humanoid balance and orientation tracking.

### IMU Sensor Definition

```xml
<gazebo reference="imu_link">
  <sensor name="imu" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/humanoid</namespace>
        <argument>~/out:=imu</argument>
      </ros>
      <frame_name>imu_link</frame_name>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>
  </sensor>
</gazebo>
```

### Processing IMU Data

```python
from sensor_msgs.msg import Imu
import numpy as np

class IMUProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/humanoid/imu',
            self.imu_callback,
            10
        )
        
    def imu_callback(self, msg):
        # Orientation (quaternion)
        quat = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        
        # Convert to Euler angles (roll, pitch, yaw)
        roll, pitch, yaw = self.quaternion_to_euler(quat)
        
        # Angular velocity
        angular_vel = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ]
        
        # Linear acceleration
        linear_accel = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ]
        
        # Check if robot is falling
        if abs(roll) > 0.5 or abs(pitch) > 0.5:
            self.get_logger().warn('ROBOT UNSTABLE!')
            
    def quaternion_to_euler(self, quat):
        # Convert quaternion to Euler angles
        # [implementation here]
        return roll, pitch, yaw
```

## Sensor Fusion Example

Combining camera + depth + IMU for robust perception:

```python
from sensor_msgs.msg import Image, Imu
from message_filters import Subscriber, ApproximateTimeSynchronizer

class SensorFusion(Node):
    def __init__(self):
        super().__init__('sensor_fusion')
        
        # Synchronized subscribers
        image_sub = Subscriber(self, Image, '/humanoid/camera/image_raw')
        depth_sub = Subscriber(self, Image, '/humanoid/camera/depth/image_raw')
        imu_sub = Subscriber(self, Imu, '/humanoid/imu')
        
        self.sync = ApproximateTimeSynchronizer(
            [image_sub, depth_sub, imu_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.sensor_callback)
        
    def sensor_callback(self, image_msg, depth_msg, imu_msg):
        # All three sensors are now synchronized
        self.get_logger().info('Processing fused sensor data')
        
        # Use IMU to compensate for robot motion in vision
        # Use depth to get 3D positions of detected objects
        # Combine for robust environmental understanding
```

## Visualizing Sensor Data in RViz2

```bash
rviz2
```

In RViz2:
1. **Add → Camera** - View camera feed
2. **Add → LaserScan** - Visualize 2D LiDAR
3. **Add → PointCloud2** - View 3D LiDAR or depth point cloud
4. **Add → Axes** - Show IMU orientation

## Performance Considerations

> [!TIP]
> **Reduce sensor update rates** if simulation is slow:
> - Cameras: 10-30 Hz is often sufficient
> - LiDAR: 5-10 Hz for navigation
> - IMU: 100 Hz for balance control

> [!WARNING]
> **GPU acceleration required** for multiple cameras or high-resolution LiDAR. Use `gpu_ray` sensor type when available.

## Next Steps

- **[Module 3: NVIDIA Isaac Overview](../module3-isaac/overview.md)** - Hardware-accelerated perception
- Apply these sensors in navigation and manipulation tasks

## Resources

- [Gazebo Sensors](https://gazebosim.org/docs/harmonic/sensors)
- [ROS 2 Sensor Messages](https://github.com/ros2/common_interfaces/tree/rolling/sensor_msgs)
- [cv_bridge Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Cpp.html)
