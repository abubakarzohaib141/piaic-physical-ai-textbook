# 13-Week Course Outline

## Course Structure

This course follows a **progressive, hands-on approach** from foundations to deployment. Each week builds upon the previous, culminating in an **autonomous humanoid robot capstone project**.

---

## Weeks 1-2: Introduction to Physical AI

### Week 1: Foundations of Physical AI

**Learning Objectives:**
- Understand the shift from digital AI to embodied intelligence
- Learn the hardware requirements for robotics development
- Set up development environment

**Topics:**
- What is Physical AI? Digital vs. Physical Intelligence
- The humanoid robot landscape (Atlas, Digit, Figure 01, Unitree)
- Sensor systems overview: LiDAR, cameras, IMUs, force/torque sensors
- Development workstation vs. edge computing

**Hands-on:**
- Install Ubuntu 22.04 LTS
- Set up ROS 2 Humble
- Configure development environment (VS Code with ROS extensions)

**Assignment:**
- Research paper: Compare 3 commercial humanoid robots
- Lab: Verify ROS 2 installation with sample nodes

---

### Week 2: Robot Sensors and Perception

**Learning Objectives:**
- Understand sensor types and their applications
- Process camera and LiDAR data
- Learn coordinate frame transformations

**Topics:**
- Camera fundamentals: RGB, depth, stereo
- LiDAR: 2D vs 3D, point clouds
- IMU: Accelerometer, gyroscope, magnetometer
- tf2: Transform library for coordinate frames

**Hands-on:**
- Subscribe to camera topics and display images
- Process LiDAR scans for obstacle detection
- Visualize transforms in RViz2

**Assignment:**
- Lab: Build a wall-following robot using LiDAR
- Quiz: Sensor selection for different tasks

---

## Weeks 3-5: ROS 2 Fundamentals

### Week 3: Nodes, Topics, and Communication

**Learning Objectives:**
- Create ROS 2nodes in Python and C++
- Implement publisher-subscriber patterns
- Use ROS 2 command-line tools

**Topics:**
- Node lifecycle and best practices
- Quality of Service (QoS) policies
- Standard message types (geometry_msgs, sensor_msgs)
- rqt tools for debugging

**Hands-on:**
- Create a camera processor node
- Build a sensor fusion node (camera + LiDAR)
- Debug communication with rqt_graph

**Assignment:**
- Project: Multi-sensor perception pipeline
- Reading: ROS 2 Design Concepts

---

### Week 4: Services, Actions, and Parameters

**Learning Objectives:**
- Implement request-response services
- Use actions for long-running tasks
- Manage node parameters dynamically

**Topics:**
- Service vs. topic vs. action: When to use each
- Creating custom service definitions
- Action servers and clients  
- Dynamic parameter reconfiguration

**Hands-on:**
- Build a "move to pose" action server
- Create a calibration service
- Use parameters for sensor configuration

**Assignment:**
- Lab: Implement a robotic arm pick-and-place action
- Deliverable: ROS 2 package with services and actions

---

### Week 5: URDF and Robot Description

**Learning Objectives:**
- Write URDF files for robot structure
- Use Xacro for modular robot definitions
- Visualize robots in RViz2

**Topics:**
- URDF: Links, joints, visual, collision, inertial
- Joint types: revolute, continuous, prismatic, fixed
- Xacro macros for code reuse
- Calculating inertial properties

**Hands-on:**
- Build a simple humanoid URDF (5-DOF)
- Use joint_state_publisher to control joints
- Export from SolidWorks to URDF

**Assignment:**
- Project: Create URDF for a 12-DOF humanoid
- Challenge: Add sensors (camera, IMU) to URDF

---

## Weeks 6-7: Robot Simulation with Gazebo

### Week 6: Gazebo Fundamentals

**Learning Objectives:**
- Set up Gazebo simulation environments
- Spawn robots from URDF
- Configure physics parameters

**Topics:**
- SDF world files
- Physics engines: ODE, Bullet, Simbody
- Material properties and friction
- Gazebo plugins (camera, LiDAR, IMU)

**Hands-on:**
- Create a custom world with obstacles
- Spawn humanoid robot in simulation
- Configure joint controllers

**Assignment:**
- Lab: Build a warehouse environment
- Deliverable: Simulated robot navigating environment

---

### Week 7: Sensor Simulation and Control

**Learning Objectives:**
- Simulate cameras, LiDAR, IMU in Gazebo
- Integrate ros2_control for robot actuation
- Test control algorithms in simulation

**Topics:**
- Gazebo sensor plugins
- ros2_control architecture
- Position, velocity, and effort controllers
- Sensor noise and realism

**Hands-on:**
- Add depth camera to humanoid
- Implement joint position controllers
- Collect simulated sensor data

**Assignment:**
- Project: Teleoperated humanoid with camera feed
- Reading: Sim-to-real transfer challenges

---

## Weeks 8-10: NVIDIA Isaac Platform

### Week 8: Isaac Sim and Omniverse

**Learning Objectives:**
- Set up NVIDIA Isaac Sim
- Import robots into Isaac Sim
- Generate synthetic training data

**Topics:**
- USD (Universal Scene Description) format
- PhysX physics engine
- RTX ray-traced rendering
- Domain randomization for sim-to-real

**Hands-on:**
- Load humanoid in Isaac Sim
- Create photorealistic environments
- Generate annotated image dataset

**Assignment:**
- Lab: Collect 10,000 images with segmentation labels
- Challenge: Implement domain randomization

---

### Week 9: Isaac ROS - Hardware Acceleration

**Learning Objectives:**
- Use Isaac ROS GEMs for perception
- Implement visual SLAM
- Accelerate inference with TensorRT

**Topics:**
- Isaac ROS Visual SLAM
- Isaac ROS DNN Inference
- Isaac ROS AprilTag detection
- Image segmentation with GPU

**Hands-on:**
- Run VSLAM on simulated humanoid
- Deploy object detection (YOLO) on GPU
- Benchmark CPU vs. GPU performance

**Assignment:**
- Project: Real-time object tracking with camera
- Report: Performance comparison analysis

---

### Week 10: Isaac Gym - Reinforcement Learning

**Learning Objectives:**
- Train policies with Isaac Gym
- Implement PPO for bipedal walking
- Scale training with parallel environments

**Topics:**
- Massively parallel simulation (4096+ envs)
- PPO, SAC, TD3 algorithms
- Observation and action spaces for humanoids
- Reward shaping for stable walking

**Hands-on:**
- Train walking policy for simple biped
- Visualize training progress
- Deploy trained policy to Isaac Sim

**Assignment:**
- Project: Train humanoid to walk forward
- Bonus: Train to walk on uneven terrain

---

## Weeks 11-12: Humanoid Robot Development

### Week 11: Kinematics and Locomotion

**Learning Objectives:**
- Understand humanoid kinematics
- Implement inverse kinematics (IK)
- Generate stable walking gaits

**Topics:**
- Forward/inverse kinematics
- Zero Moment Point (ZMP) for balance
- Footstep planning
- Whole-body control

**Hands-on:**
- Solve IK for arm reaching  
- Implement ZMP-based walk planner
- Test gaits in simulation

**Assignment:**
- Lab: Humanoid walks to target position
- Reading: Boston Dynamics Atlas locomotion

---

### Week 12: Manipulation and Interaction

**Learning Objectives:**
- Plan manipulation tasks
- Implement grasping algorithms
- Design human-robot interaction patterns

**Topics:**
- Grasp planning and execution
- MoveIt2 for motion planning
- Compliance and force control
- Natural interaction modalities

**Hands-on:**
- Pick and place objects in simulation
- Implement compliant control
- Design gesture-based commands

**Assignment:**
- Project: Humanoid picks up multiple objects
- Demo: Interactive robot behavior

---

## Week 13: Vision-Language-Action & Capstone

### Topics:
- Integrating GPT-4 with robot control
- OpenAI Whisper for voice commands
- End-to-end VLA pipeline
- Multimodal perception and reasoning

### Capstone Project: The Autonomous Humanoid

**Requirements:**

Your robot must:

1. **Voice Command Input**
   - Listen for natural language commands
   - Use OpenAI Whisper for speech-to-text
   - Example: "Bring me the red cup from the kitchen"

2. **Task Planning**
   - Use GPT-4 to decompose commands
   - Generate action sequences
   - Handle ambiguity with clarifying questions

3. **Navigation**
   - Plan collision-free paths
   - Navigate to specified locations
   - Use Nav2 or custom planner

4. **Perception**
   - Identify objects using computer vision
   - Segment environment (floor, obstacles, objects)
   - Track objects in 3D space

5. **Manipulation**
   - Approach and grasp target object
   - Execute compliant motion
   - Place object safely

6. **Feedback & Confirmation**
   - Provide visual feedback (LED, display)
   - Speak status updates (TTS)
   - Handle errors gracefully

**Deliverables:**

- ✅ Fully functional ROS 2 workspace
- ✅ Simulated humanoid in Gazebo/Isaac Sim
- ✅ Voice-to-action pipeline implementation
- ✅ 5-minute video demonstration
- ✅ Technical report (10 pages max)
- ✅ Code repository with documentation

**Evaluation Criteria** (100 points):

| Category | Points | Criteria |
|----------|--------|----------|
| **Functionality** | 40 | Does it work end-to-end? |
| **Code Quality** | 20 | Clean, documented, follows ROS 2 best practices |
| **Innovation** | 15 | Novel approaches or features |
| **Robustness** | 15 | Error handling, edge cases |
| **Presentation** | 10 | Clear demo and documentation |

**Bonus Challenges** (+20 points each):

- 🏆 **Sim-to-Real Transfer**: Deploy to physical robot (Jetson + sensors)
- 🏆 **Multi-Object Task**: "Clear the table" - handle multiple objects
- 🏆 **Dynamic Obstacles**: Navigate around moving people
- 🏆 **Multi-Turn Dialog**: "Where is the cup?" → "Move it to the table"

---

## Assessment Summary

### Throughout the Course:

- **Weekly Labs** (30%): Hands-on exercises
- **Quizzes** (10%): Conceptual understanding
- **Module Projects** (30%): 4 major projects (ROS 2, Gazebo, Isaac, VLA)
- **Capstone** (30%): Final autonomous humanoid project

### Grading Scale:

- A: 90-100% - Exceptional work with innovation
- B: 80-89% - Solid understanding and implementation
- C: 70-79% - Basic competency achieved
- D: 60-69% - Incomplete understanding
- F: <60% - Did not meet requirements

---

## Prerequisites

**Required:**
- Python programming proficiency
- Basic Linux command line
- Linear algebra (matrices, vectors)

**Recommended:**
- C++ knowledge (for performance-critical code)
- Computer vision basics
- Control theory fundamentals

---

## Required Hardware

**Must Have:**
- Workstation with NVIDIA RTX GPU (4070 Ti or better)
- 64GB RAM, Ubuntu 22.04 LTS

**Highly Recommended:**
- NVIDIA Jetson Orin Nano kit (~$700)
- Intel RealSense D435i camera
- USB microphone for voice commands

**Optional (for sim-to-real):**
- Unitree Go2 quadruped OR
- Unitree G1 humanoid OR
- Custom built robot platform

---

## Learning Resources

### Official Documentation:
- [ROS 2 Humble Docs](https://docs.ros.org/en/humble/)
- [Gazebo Documentation](https://gazebosim.org/docs)
- [NVIDIA Isaac Docs](https://docs.omniverse.nvidia.com/isaacsim/latest/)

### Recommended Textbooks:
- *Introduction to Autonomous Mobile Robots* (Siegwart & Nourbakhsh)
- *Probabilistic Robotics* (Thrun, Burgard, Fox)
- *Modern Robotics* (Lynch & Park)

### Online Courses:
- Coursera: Robotics Specialization (UPenn)
- Udacity: Robotics Nanodegree
- YouTube: The Construct (ROS 2 tutorials)

---

## Tips for Success

> [!TIP]
> **Start Early**: Don't wait until the last minute for capstone project. Begin thinking about it from Week 1.

> [!IMPORTANT]
> **Document Everything**: Keep a lab notebook. Future you will thank current you.

> [!WARNING]
> **Simulation ≠ Reality**: Always test assumptions. What works in Gazebo may fail on hardware.

---

## Office Hours & Support

- **Instructor Office Hours**: Tuesdays & Thursdays, 2-4 PM
- **TA Lab Sessions**: Mondays & Wednesdays, 6-8 PM
- **Online Forum**: Discourse (24/7 peer support)
- **Emergency Contact**: For hardware issues or critical bugs

---

## Future Pathways

After completing this course, you'll be prepared for:

- **Research**: Humanoid locomotion, manipulation, HRI
- **Industry**: Robotics engineer at Boston Dynamics, Tesla, Figure AI
- **Entrepreneurship**: Start your own robotics company
- **Advanced Study**: PhD in robotics, AI, or mechatronics

---

**Ready to build the future? Let's begin!** 🤖

[← Back to Introduction](../introduction.md)
