# Python-ROS2 Control System 

This part of the project serves as the decision-making and control component of the autonomous vehicle system. It complements the Webots simulation by implementing algorithms for Adaptive Cruise Control (ACC), leveraging ROS 2 for communication and control.

## Features

- **Adaptive Cruise Control (ACC)**: Dynamically adjusts vehicle speed to maintain a safe distance from the vehicle ahead.

- **ROS 2 Integration**: Communicates with the Webots simulation via ROS 2 topics for seamless sensor data processing and control commands.

- **Modular Design**: Organized into reusable components for easy maintenance and scalability.

## Directory Structure

```plaintext
Grad_ws/
├── src/vehicle_controller/
│       ├── Launch/
│       │   └── Sim_launch.py           # Simulation Launch file
│       ├── vehicle_controller/
│       │   └── vehicle_controller.py   # Controller Entry point
│       ├── package.xml
│       ├── setup.cfg
│       └── setup.py
└── README.md
```

## How to Run

1. Build the ROS 2 Workspace:

    ```bash
    cd $GP_WS_DIR/Grad_ws/
    colcon build --symlink-install
    ```

2. Set Up the Environment:

    ```bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "source $GP_WS_DIR/Grad_ws/install/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

3. Launch the Simulation:

    ```bash
    ros2 launch vehicle_controller Sim_launch.py
    ```

4. After the launch logs a **"Ready"** message, you can reset and run the webots simulation.

5. **Feel free to edit and try your own algorithm!!**
