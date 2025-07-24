# Webots-ROS2 Autonomous Vehicle

This Part of the project serves as the foundation for simulating the vehicle side of an autonomous driving system using Webots and ROS 2. It focuses on the simulation of vehicle dynamics, sensor integration, and control interfaces. A [complementary system](../Grad_ws/) will host the code and algorithms required to demonstrate the decision-making and control aspects of the autonomous vehicle, ensuring a complete and functional simulation environment.

## Features

- **LiDAR and WheelSpeed Integration**: Real-time data from sensors for perception.

- **Vehicle Control**: Command-based control for steering, velocity, and braking.

- **ROS 2 Compatibility**: Fully integrated with ROS 2 Humble for modular and scalable development.

- **Webots Simulation**: High-fidelity simulation of autonomous vehicle behavior in custom environments.

## ROS 2 Topics

### ros2av: Ego vehicle's controller - Publishers and Subscribers

<div style="display: flex; justify-content: space-between;">

<div style="width: 48%;">
<strong>Publishers</strong>

| Topic Name | Message Type | Description |
| --- | --- | --- |
| `/wbts_time` | `std_msgs/Float64` | Simulation timestamp |
| `/get_vel` | `std_msgs/Float64` | Ego Vehicle's Velocity<br>Used for plots only |
| `/av_lidar` | `sensor_msgs/LaserScan` | LiDAR data |
| `/wheel_speed` | `example_interfaces`<br>`/Float64MultiArray` | Right and left wheel speed data |
</div>

<div style="width: 48%;">
<strong>Subscribers</strong>

| Topic Name | Message Type | Notes |
| --- | --- | --- |
| `/cmd_vel` | `std_msgs/Float64` | Range `(-20.0 : 100)` |
| `/SteeringAngle` | `std_msgs/Float64` | Range `(-0.5 : 0.5)` |
| `/brakes` | `std_msgs/Float64` | Range `(0 : 1)` |
</div>

</div>

### drive_cycle: Lead vehicle's controller - Publishers and Subscribers

<div style="display: flex; justify-content: space-between;">

<div style="width: 48%;">
<strong>Publishers</strong>

| Topic Name | Message Type | Description |
| --- | --- | --- |
| `/lead_vel` | `std_msgs/Float32` | Lead Vehicle's Velocity<br>Used for plots only |
</div>

<div style="width: 48%;">
<strong>Subscribers</strong>

| Topic Name | Message Type | Notes |
| --- | --- | --- |
| `/pause` | `std_msgs/Bool` | `msg.data` is neglected |
| `/realtime` | `std_msgs/Bool` | `msg.data` is neglected |
| `/fast` | `std_msgs/Bool` | `msg.data` is neglected |
</div>

</div>

## Directory Structure

The `ros2av` directory contains the main python script that manages the autonomous vehicle. Below is an overview of the overall structure:
```plaintext
WBs/
├── controllers/        # Contains custom Webots controllers for simulating vehicle behavior.
│   ├── drive_cycle/        # Lead_Vehicle's Controller directory.
│   └── ros2av/             # Ego_Vehicle's Controller directory.
├── plugins/
├── worlds/             # Houses Webots world files.
│   ├── city.wbt            # Original map (deprecated)
│   └── RoadTest.wbt        << Current Pick
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
    cd $GP_WS_DIR/WBs/worlds/
    webots RoadTest.wbt
    ```

3. Pause the simulation for now

3. Continue to [**ROS2 Setup**](../Grad_ws/README.md#how-to-run) instructions.
