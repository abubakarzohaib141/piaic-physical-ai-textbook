# Module 3: The AI-Robot Brain - NVIDIA Isaac

## Overview

NVIDIA Isaac is the industry-leading platform for developing, testing, and deploying AI-powered robots. It provides:

- **Isaac Sim**: Photorealistic physics simulation
- **Isaac ROS**: Hardware-accelerated perception and navigation
- **Isaac Gym**: Massively parallel reinforcement learning
- **Omniverse**: Collaborative 3D workflow platform

## Why NVIDIA Isaac for Humanoid Robots?

Traditional robotics pipelines are CPU-bound and struggle with:
- Real-time computer vision (30+ FPS)
- SLAM with high-resolution sensors
- Multi-robot simulation at scale

**Isaac leverages GPU acceleration** to achieve:
- ✅ **10-100x faster** perception vs. CPU
- ✅ **Photorealistic rendering** for sim-to-real transfer
- ✅ **Thousands of parallel environments** for RL training
- ✅ **RTX ray tracing** for accurate sensor simulation

## Isaac Sim - The Digital Twin on Steroids

### Installation

```bash
# Download Omniverse Launcher
# https://www.nvidia.com/en-us/omniverse/download/

# Install Isaac Sim from Omniverse Launcher
# Version: 2023.1.1 or later

# Install ROS 2 bridge
sudo apt install ros-humble-isaac-ros-common
```

### Key Features

#### 1. USD (Universal Scene Description)

Isaac Sim uses Pixar's USD format for scenes:

```python
from pxr import Usd, UsdGeom, Gf

# Create a scene
stage = Usd.Stage.CreateNew("humanoid_scene.usd")

# Add a sphere (head)
sphere = UsdGeom.Sphere.Define(stage, "/World/head")
sphere.GetRadiusAttr().Set(0.15)
sphere.AddTranslateOp().Set(Gf.Vec3f(0, 0, 1.7))

# Save
stage.Save()
```

#### 2. PhysX Physics Engine

NVIDIA PhysX provides real-time physics with:
- Articulation solver for robotic joints
- Contact and friction simulation
- Soft body dynamics
- GPU-accelerated cloth simulation

#### 3. RTX Rendering

Enables photorealistic sensor simulation:
- Ray-traced lighting and shadows
- Accurate depth and semantic segmentation
- Material properties (metallic, transparent, etc.)

### Loading a Humanoid in Isaac Sim

```python
from omni.isaac.kit import SimulationApp

# Initialize
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.prims import create_prim

# Create world
world = World()
world.scene.add_default_ground_plane()

# Load humanoid from USD
robot = Robot(
    prim_path="/World/humanoid",
    name="my_humanoid",
    usd_path="/path/to/humanoid.usd"
)

# Add to scene
world.scene.add(robot)

# Reset and play
world.reset()
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
```

## Isaac ROS - Hardware-Accelerated Perception

### Architecture

```mermaid
graph LR
    A[Camera/Sensor] --> B[Isaac ROS GEM]
    B --> C[GPU Processing]
    C --> D[ROS 2 Topic]
    D --> E[Navigation/Planning]
```

### Key Packages

#### 1. Isaac ROS Visual SLAM

Real-time VSLAM using GPU:

```bash
# Install
sudo apt install ros-humble-isaac-ros-visual-slam

# Launch
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py
```

```python
# Configuration
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='isaac_ros_visual_slam',
            executable='isaac_ros_visual_slam',
            parameters=[{
                'denoise_input_images': True,
                'rectified_images': True,
                'enable_imu_fusion': True,
                'gyro_noise_density': 0.000244,
                'gyro_random_walk': 0.000019393,
                'accel_noise_density': 0.001862,
                'accel_random_walk': 0.003,
            }],
            remappings=[
                ('stereo_camera/left/image', '/humanoid/camera/left/image_raw'),
                ('stereo_camera/right/image', '/humanoid/camera/right/image_raw'),
                ('visual_slam/imu', '/humanoid/imu'),
            ]
        )
    ])
```

#### 2. Isaac ROS DNN Inference

Hardware-accelerated deep learning:

```bash
# Object detection with DOPE (Deep Object Pose Estimation)
ros2 launch isaac_ros_dope isaac_ros_dope_tensor_rt.launch.py
```

#### 3. Isaac ROS Depth Segmentation

Semantic segmentation from depth:

```python
# Process depth image
from isaac_ros_depth_segmentation import DepthSegmentationNode

class HumanoidPerception(Node):
    def __init__(self):
        super().__init__('perception')
        
        # Subscribe to depth
        self.depth_sub = self.create_subscription(
            Image,
            '/humanoid/camera/depth',
            self.depth_callback,
            10
        )
        
        # Publish segmented obstacles
        self.obstacle_pub = self.create_publisher(
            PointCloud2,
            '/perception/obstacles',
            10
        )
```

## Isaac Gym - Massively Parallel RL

Train policies with thousands of parallel environments:

```python
from isaacgym import gymapi, gymutil

# Create gym instance
gym = gymapi.acquire_gym()

# Create simulation
sim_params = gymapi.SimParams()
sim_params.use_gpu_pipeline = True
sim = gym.create_sim(0, 0, gymapi.SIM_PHYSX, sim_params)

# Create 4096 parallel environments
num_envs = 4096
envs_per_row = int(np.sqrt(num_envs))

for i in range(num_envs):
    env = gym.create_env(sim, lower, upper, envs_per_row)
    # Load humanoid asset
    asset = gym.load_asset(sim, asset_root, asset_file)
    actor = gym.create_actor(env, asset, pose, f"humanoid_{i}", i, 0)
    
# Simulate
while True:
    gym.simulate(sim)
    gym.fetch_results(sim, True)
    gym.step_graphics(sim)
    gym.render_all_camera_sensors(sim)
```

### Training Humanoid Walking

```python
import torch
import torch.nn as nn

class HumanoidPolicyNetwork(nn.Module):
    def __init__(self, obs_dim, act_dim):
        super().__init__()
        self.fc1 = nn.Linear(obs_dim, 256)
        self.fc2 = nn.Linear(256, 128)
        self.fc3 = nn.Linear(128, act_dim)
        
    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        return torch.tanh(self.fc3(x))

# Observations: joint positions, velocities, IMU
# Actions: joint torques
obs_dim = 37  # 12 joints * 2 + IMU (9) + base velocity (4)
act_dim = 12  # 12 joint torques

policy = HumanoidPolicyNetwork(obs_dim, act_dim).cuda()

# Train with PPO, SAC, etc.
```

## Synthetic Data Generation

Generate training data for computer vision:

```python
from omni.isaac.synthetic_utils import SyntheticDataHelper

# In Isaac Sim
sd_helper = SyntheticDataHelper()

# Enable RGB, depth, semantic segmentation, instance segmentation
sd_helper.enable_sensors(
    camera_path="/World/humanoid/head/camera",
    rgb=True,
    depth=True,
    semantic=True,
    instance=True
)

# Collect data
for i in range(10000):
    world.step()
    
    rgb = sd_helper.get_rgb()
    depth = sd_helper.get_depth()
    semantic = sd_helper.get_semantic_segmentation()
    
    # Save for training
    save_data(rgb, depth, semantic, f"frame_{i}")
```

## Sim-to-Real Transfer

### Domain Randomization

```python
from omni.isaac.core.utils.prims import get_prim_at_path
from pxr import UsdShade

# Randomize lighting
for i in range(num_lights):
    light_prim = get_prim_at_path(f"/World/Light_{i}")
    light_prim.GetAttribute("intensity").Set(random.uniform(500, 3000))
    
# Randomize materials
for obj in scene_objects:
    material = UsdShade.Material.Get(stage, obj.material_path)
    material.GetInput("roughness").Set(random.uniform(0.1, 0.9))
    material.GetInput("metallic").Set(random.uniform(0.0, 1.0))
    
# Randomize robot dynamics
robot_prim = get_prim_at_path("/World/humanoid")
for joint in robot_prim.joints:
    joint.friction = random.uniform(0.01, 0.1)
    joint.damping = random.uniform(0.1, 1.0)
```

## Cloud Deployment

Scale training with Isaac Sim on cloud:

```bash
# Using NVIDIA NGC Containers
docker pull nvcr.io/nvidia/isaac-sim:2023.1.1

# Run headless
docker run --gpus all -it \
  -v $(pwd)/workspace:/workspace \
  nvcr.io/nvidia/isaac-sim:2023.1.1 \
  /isaac-sim/python.sh /workspace/train_humanoid.py
```

## Best Practices

> [!TIP]
> **Start Simple**: Begin with basic walking before attempting complex tasks like object manipulation.

> [!WARNING]
> **GPU Memory**: Isaac Sim + Isaac Gym can consume 20GB+ VRAM with large parallel environments.

> [!IMPORTANT]
> **Sim-to-Real Gap**: Always test trained policies in simulation with noise and randomization before deploying to real hardware.

## Next Steps

- **[Navigation with Nav2](navigation.md)** - Path planning for humanoids
- **[Module 4: VLA](../module4-vla/overview.md)** - Language-driven robot control

## Resources

- [Isaac Sim Docs](https://docs.omniverse.nvidia.com/isaacsim/latest/)
- [Isaac ROS](https://nvidia-isaac-ros.github.io/)
- [Isaac Gym](https://developer.nvidia.com/isaac-gym)
- [Omniverse](https://www.nvidia.com/en-us/omniverse/)
