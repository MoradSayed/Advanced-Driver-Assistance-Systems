# Webots-ROS2 Autonomous Vehicle

This Part of the project serves as the foundation for simulating the vehicle side of an autonomous driving system using Webots and ROS 2. It focuses on the simulation of vehicle dynamics, sensor integration, and control interfaces. A [complementary system](../Grad_ws/) will host the code and algorithms required to demonstrate the decision-making and control aspects of the autonomous vehicle, ensuring a complete and functional simulation environment.

## Features

- **Camera and LiDAR Integration**: Real-time data from sensors for perception.

- **Vehicle Control**: Command-based control for steering, velocity, and braking.

- **ROS 2 Compatibility**: Fully integrated with ROS 2 Humble for modular and scalable development.

- **Webots Simulation**: High-fidelity simulation of autonomous vehicle behavior in custom environments.

## ROS 2 Topics

### Publishers and Subscribers

<div style="display: flex; justify-content: space-between;">

<div style="width: 48%;">
<strong>Publishers</strong>
<table>
<thead>
<tr>
<th>Topic Name</th>
<th>Message Type</th>
</tr>
</thead>
<tbody>
<tr>
<td><code>/av_camera</code></td>
<td><code>sensor_msgs/Image</code></td>
</tr>
<tr>
<td><code>/av_lidar</code></td>
<td><code>sensor_msgs/LaserScan</code></td>
</tr>
</tbody>
</table>
</div>

<div style="width: 48%;">
<strong>Subscribers</strong>
<table>
<thead>
<tr>
<th>Topic Name</th>
<th>Message Type</th>
<th>Range</th>
</tr>
</thead>
<tbody>
<tr>
<td><code>/cmd_vel</code></td>
<td><code>std_msgs/Float64</code></td>
<td><code>(-20 : 60)</code></td>
</tr>
<tr>
<td><code>/SteeringAngle</code></td>
<td><code>std_msgs/Float64</code></td>
<td><code>(-0.5 : 0.5)</code></td>
</tr>
<tr>
<td><code>/brakes</code></td>
<td><code>std_msgs/Float64</code></td>
<td><code>(0 : 1)</code></td>
</tr>
</tbody>
</table>
</div>

</div>

## Directory Structure

The `ros2av` directory contains the main python script that manages the autonomous vehicle. Below is an overview of the overall structure:
```plaintext
WBs/
├── controllers/  # Contains custom Webots controllers for simulating vehicle behavior.
│   └── ros2av/       # Our Custom Controller directory.
├── plugins/      # Includes Webots plugins for extending simulation capabilities.
├── worlds/       # Houses Webots world files.
│   └── city.wbt      # Webots world file.
└── README.md
```

## How to Run

1. **Setup Environment**:
    Ensure you have ROS 2 Humble and Webots installed. Source the ROS 2 workspace:
    ```bash
    source /opt/ros/humble/setup.bash
    ```
2. **Launch the Simulation**: 
    Navigate to the worlds directory and launch the simulator:
    ```bash
    cd Advanced-Driver-Assistance-Systems/WBs/worlds/
    webots city.wbt
    ```

3. **Continue to [ROS2 Setup](../Grad_ws/README.md) instructions.**
