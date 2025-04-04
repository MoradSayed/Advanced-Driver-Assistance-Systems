<div align="center">
  <center><h1>Advanced Driver-Assistance Systems</h1></center>
</div>

## Overview

This project is developed to simulate and implement advanced driver-assistance systems (ADAS) using [Webots](https://cyberbotics.com/) and [ROS 2](https://docs.ros.org/en/humble/index.html). The primary focus is on systems such as Adaptive Cruise Control (ACC) and Automatic Emergency Braking (AEB), aiming to enhance vehicle safety and autonomy.

## Features

- **Adaptive Cruise Control (ACC):** Maintains a safe distance from the vehicle ahead by adjusting the vehicle's speed.

- **Automatic Emergency Braking (AEB):** Detects imminent collisions and applies brakes automatically to prevent accidents.

- **Integration with Webots:** Utilizes the Webots open-source 3D simulator for accurate vehicle dynamics and environment simulation.

- **ROS 2 Compatibility:** Leverages the Robot Operating System (ROS) 2 framework for modularity and scalability.

## Project Structure

```
ADAS
├── Grad_ws    # Ros2 workspace
├── WBs        # Webots environment
├── LICENSE
└── README.md
```

## Installation (Ubuntu 22.04)

### Prerequisites

- **Webots:** Install version **R2025a** from [Github Releases](https://github.com/cyberbotics/webots/releases/tag/R2025a).
- **ROS 2:** Follow the ROS 2 **Humble** [installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
- **webots_ros2:** Used to provides an interface between ROS 2 and Webots. Install from [here](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html#background).
- **Python3.10:** Use your preferred installation method, or follow this [tutorial](https://tutorpython.com/install-python-3-10-on-ubuntu-22-04#Prerequisites_for_installing_Python_310_on_Ubuntu_2204).

### Setup Instructions

1. **Clone the Repository:**

    ```bash
    git clone git@github.com:MoradSayed/Advanced-Driver-Assistance-Systems.git
    ```

2. **Follow [Webots World](/WBs/README.md#how-to-run) Instructions.**

3. **Follow [ROS2 WorkSpace](/Grad_ws/README.md#) Instructions.**

## License

This project is [licensed](LICENSE) under the [BSD 3-Clause License](https://opensource.org/license/bsd-3-clause).
