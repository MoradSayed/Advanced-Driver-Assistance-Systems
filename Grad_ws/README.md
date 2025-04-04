# Python-ROS2 Control System 

This part of the project serves as the decision-making and control component of the autonomous vehicle system. It complements the Webots simulation by implementing algorithms for Adaptive Cruise Control (ACC) and Automatic Emergency Braking (AEB), leveraging ROS 2 for communication and control.

## Features

- **Adaptive Cruise Control (ACC)**: Dynamically adjusts vehicle speed to maintain a safe distance from the vehicle ahead.

- **Automatic Emergency Braking (AEB)**: Detects potential collisions and triggers braking to prevent accidents.

- **ROS 2 Integration**: Communicates with the Webots simulation via ROS 2 topics for seamless sensor data processing and control commands.

- **Modular Design**: Organized into reusable components for easy maintenance and scalability.

## Directory Structure

```plaintext
Grad_ws/
├── src/
│   └── vehicle_controller/
│       ├── vehicle_controller/
│       │   ├── vehicle_controller.py   # Controller Entry point
│       │   └── ...
│       ├── package.xml
│       └── setup.py
└── README.md
```

## How to Run

1. **Build the ROS 2 Workspace:**

    ```bash
    cd <REPO_ROOT>/Grad_ws/         # Replace <REPO_ROOT> with the location of the cloned repo
    colcon build --symlink-install
    ```

2. **Set Up the Environment:**

    ```bash
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    
    # Optional: Permanently add the sources to .bashrc
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "source <REPO_ROOT>/Grad_ws/install/setup.bash" >> ~/.bashrc   # Replace <REPO_ROOT>
    source ~/.bashrc
    ```

3. **Launch the Simulation:**

    ```bash
    ros2 launch vehicle_controller vehicle_controller
    ```
4. **Feel free to edit and try your own algorithm!!**