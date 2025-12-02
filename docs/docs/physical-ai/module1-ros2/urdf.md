# URDF - Unified Robot Description Format

## What is URDF?

The **Unified Robot Description Format** (URDF) is an XML-based file format for describing a robot's physical structure. Think of it as the blueprint that tells ROS 2 (and simulation environments) about:

- **Links**: Rigid bodies (torso, arms, legs, head)
- **Joints**: Connections between links (revolute, prismatic, fixed)
- **Visual Properties**: What the robot looks like
- **Collision Properties**: Simplified geometry for physics calculations
- **Inertial Properties**: Mass, center of mass, inertia tensors

## Why URDF Matters for Humanoid Robots

Humanoid robots are **kinematically complex**:
- 30+ degrees of freedom (DOF)
- Multiple kinematic chains (arms, legs, torso)
- Complex joint dependencies
- Weight distribution affects balance

URDF provides:
✅ **Standardized representation** across simulators (Gazebo, Isaac Sim)  
✅ **Automatic forward/inverse kinematics** calculation  
✅ **Collision detection** and physics simulation  
✅ **Visualization** in RViz2

## Anatomy of a URDF File

### Basic Structure

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  
  <!-- Links (rigid bodies) -->
  <link name="base_link">
    <visual>...</visual>
    <collision>...</collision>
    <inertial>...</inertial>
  </link>
  
  <!-- Joints (connections) -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="upper_arm"/>
    <origin xyz="0.15 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
  </joint>
  
</robot>
```

### Links in Detail

```xml
<link name="torso">
  <!-- Visual: What you see in RViz/Gazebo -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.3 0.2 0.4"/>  <!-- width, depth, height -->
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>  <!-- R G B Alpha -->
    </material>
  </visual>
  
  <!-- Collision: Simplified geometry for physics -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.3 0.2 0.4"/>
    </geometry>
  </collision>
  
  <!-- Inertial: Mass and inertia properties -->
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="5.0"/>  <!-- kg -->
    <inertia 
      ixx="0.1" ixy="0.0" ixz="0.0"
      iyy="0.1" iyz="0.0"
      izz="0.1"/>
  </inertial>
</link>
```

#### Geometry Types

```xml
<!-- Box -->
<geometry>
  <box size="x y z"/>
</geometry>

<!-- Cylinder -->
<geometry>
  <cylinder radius="0.05" length="0.3"/>
</geometry>

<!-- Sphere -->
<geometry>
  <sphere radius="0.1"/>
</geometry>

<!-- Mesh (from CAD file) -->
<geometry>
  <mesh filename="package://my_robot/meshes/torso.stl" scale="1.0 1.0 1.0"/>
</geometry>
```

### Joints in Detail

#### Joint Types

| Type | Description | Example Use |
|------|-------------|-------------|
| **revolute** | Rotating hinge with limits | Elbow, knee, shoulder |
| **continuous** | Rotating hinge, no limits | Wheel rotation |
| **prismatic** | Sliding linear motion | Telescoping arm |
| **fixed** | No motion | Camera mount, sensors |
| **floating** | 6-DOF free motion | Rarely used |
| **planar** | 2D motion in a plane | Mobile base (sometimes) |

#### Revolute Joint Example (Elbow)

```xml
<joint name="left_elbow" type="revolute">
  <parent link="left_upper_arm"/>
  <child link="left_forearm"/>
  
  <!-- Position and orientation relative to parent -->
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  
  <!-- Axis of rotation (Y-axis = pitch) -->
  <axis xyz="0 1 0"/>
  
  <!-- Joint limits -->
  <limit 
    lower="0.0"        <!-- 0 degrees (straight) -->
    upper="2.356"      <!-- 135 degrees (bent) -->
    effort="50.0"      <!-- Maximum torque (Nm) -->
    velocity="2.0"/>   <!-- Maximum speed (rad/s) -->
    
  <!-- Joint dynamics (optional) -->
  <dynamics damping="0.7" friction="0.0"/>
</joint>
```

## Complete Humanoid Example (Simplified)

Let's build a simple humanoid from bottom to top:

```xml
<?xml version="1.0"?>
<robot name="mini_humanoid">
  
  <!-- Base Link (pelvis/hip) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.25 0.2 0.15"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.25 0.2 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3.0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.03" iyz="0" izz="0.03"/>
    </inertial>
  </link>
  
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.3 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.15" ixy="0" ixz="0" iyy="0.15" iyz="0" izz="0.05"/>
    </inertial>
  </link>
  
  <!-- Waist Joint (pelvis to torso) -->
  <joint name="waist" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.325" rpy="0 0 0"/>
  </joint>
  
  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.12"/>
      </geometry>
      <material name="skin">
        <color rgba="0.9 0.7 0.6 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.12"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
  </link>
  
  <!-- Neck Joint -->
  <joint name="neck" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.32" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>  <!-- Yaw rotation -->
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>
  
  <!-- Left Upper Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.008" ixy="0" ixz="0" iyy="0.008" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  
  <!-- Left Shoulder Joint -->
  <joint name="left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0 0.18 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>  <!-- Pitch -->
    <limit lower="-3.14" upper="3.14" effort="50" velocity="2.0"/>
  </joint>
  
  <!-- Left Forearm -->
  <link name="left_forearm">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.7"/>
      <inertia ixx="0.004" ixy="0" ixz="0" iyy="0.004" iyz="0" izz="0.0005"/>
    </inertial>
  </link>
  
  <!-- Left Elbow Joint -->
  <joint name="left_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_forearm"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.356" effort="30" velocity="2.0"/>
  </joint>
  
  <!-- Mirror for right arm... -->
  <!-- Add legs similarly... -->
  
</robot>
```

## Using URDF with ROS 2

### 1. Visualizing in RViz2

```bash
# Install joint_state_publisher_gui
sudo apt install ros-humble-joint-state-publisher-gui

# Launch with robot_state_publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat my_robot.urdf)"

# Open RViz2
rviz2
```

In RViz2:
1. Add → RobotModel
2. Fixed Frame = "base_link"
3. Use the GUI sliders to move joints

### 2. Using with Gazebo

```xml
<!-- Add gazebo-specific tags -->
<gazebo reference="torso">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>  <!-- Friction -->
  <mu2>0.2</mu2>
</gazebo>
```

### 3. Programmatic Access

```python
from urdf_parser_py.urdf import URDF

robot = URDF.from_xml_file('my_robot.urdf')

# List all joints
for joint in robot.joints:
    print(f"{joint.name}: {joint.type}")
    
# Get link by name
torso = robot.link_map['torso']
print(f"Torso mass: {torso.inertial.mass}")
```

## Advanced URDF Concepts

### Xacro (XML Macros)

For complex robots, use **xacro** to avoid repetition:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">
  
  <!-- Define constants -->
  <xacro:property name="arm_length" value="0.3"/>
  <xacro:property name="arm_radius" value="0.04"/>
  
  <!-- Macro for creating an arm -->
  <xacro:macro name="arm" params="prefix reflect">
    <link name="${prefix}_upper_arm">
      <visual>
        <geometry>
          <cylinder radius="${arm_radius}" length="${arm_length}"/>
        </geometry>
        <material name="blue"/>
      </visual>
    </link>
    
    <joint name="${prefix}_shoulder" type="revolute">
      <parent link="torso"/>
      <child link="${prefix}_upper_arm"/>
      <origin xyz="0 ${reflect * 0.18} 0.2" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-3.14" upper="3.14" effort="50" velocity="2.0"/>
    </joint>
  </xacro:macro>
  
  <!-- Instantiate both arms -->
  <xacro:arm prefix="left" reflect="1"/>
  <xacro:arm prefix="right" reflect="-1"/>
  
</robot>
```

Convert xacro to URDF:

```bash
xacro my_robot.urdf.xacro > my_robot.urdf
```

### Calculating Inertia

For a box:
```
Ixx = (1/12) * m * (h² + d²)
Iyy = (1/12) * m * (w² + d²)
Izz = (1/12) * m * (w² + h²)
```

For a cylinder (axis along z):
```
Ixx = Iyy = (1/12) * m * (3r² + h²)
Izz = (1/2) * m * r²
```

For a sphere:
```
Ixx = Iyy = Izz = (2/5) * m * r²
```

## Best Practices

> [!TIP]
> **Keep Visual and Collision Geometries Separate**: Use detailed meshes for visual, simplified shapes for collision to improve physics performance.

> [!WARNING]
> **Inertia Matters**: Incorrect inertial properties will cause unrealistic simulation behavior. Use CAD tools to calculate accurate values.

> [!IMPORTANT]
> **Follow Naming Conventions**:
> - Links: `{body_part}` (e.g., `left_forearm`)
> - Joints: `{parent}_{child}` or descriptive names (e.g., `left_elbow`)

## Tools and Resources

### MeshLab / Blender
Convert and optimize 3D meshes for URDF

### SolidWorks to URDF Exporter
Export from CAD directly to URDF

### URDF Validation
```bash
check_urdf my_robot.urdf
urdf_to_graphiz my_robot.urdf
```

## Common Issues

### Problem: Robot Falls Through Floor
**Solution**: Check collision geometries and mass properties

### Problem: Joints Don't Move in Gazebo
**Solution**: Add transmission and actuator elements

### Problem: Robot Explodes in Simulation
**Solution**: Check for overlapping collision geometries or incorrect inertial properties

## Next Steps

- **[Module 2: Gazebo Overview](../module2-gazebo/overview.md)** - Simulate your URDF robot
- **Practice**: Create a URDF for a simple 2-DOF arm
- **Advanced**: Study existing humanoid URDFs (e.g., PR2, Atlas)

## Resources

- [URDF Official Documentation](http://wiki.ros.org/urdf)
- [URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- [Xacro Tutorials](http://wiki.ros.org/xacro)
- [Gazebo URDF Extensions](https://classic.gazebosim.org/tutorials?tut=ros_urdf)
