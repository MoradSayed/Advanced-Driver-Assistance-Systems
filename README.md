<div align="center">
  <center><h1>Advanced Driver-Assistance Systems</h1></center>
</div>

## Overview

This project is developed to simulate and implement advanced driver-assistance systems (ADAS) using [Webots](https://cyberbotics.com/) and [ROS 2](https://docs.ros.org/en/humble/index.html). Currently, our primary focus is on Adaptive Cruise Control (ACC) aiming to enhance vehicle safety and autonomy.

## Features

- **Adaptive Cruise Control (ACC):** Maintains a safe distance from the vehicle ahead by adjusting the vehicle's speed.

- **Integration with Webots:** Utilizes the Webots open-source 3D simulator for accurate vehicle dynamics and environment simulation.

- **ROS 2 Compatibility:** Leverages the Robot Operating System (ROS) 2 framework for modularity and scalability.

## Project Structure

```
ADAS
├── Grad_ws           # Ros2 workspace
├── WBs               # Webots environment
├── LICENSE
├── README.md
└── requirements.txt  # Contains required python dependencies
```

## Installation (Windows10/11, Ubuntu22.04)

> [!IMPORTANT]  
> The installation steps below have been tested only on **Ubuntu 22.04** and **Windows 11 with WSL2**.  
> They may also work on **Windows 10**, provided your OS Build is **19041 or higher** to support WSL2.  
> If you're using a different OS or environment, you may need to adapt the steps accordingly.  

### 1. Specific Prerequisites

<details>
<summary><b>For Ubuntu 22.04</b></summary>

- There's no steps in this section, you can continue to the next step.

</details>

<details>
<summary><b>For Windows 10/11</b></summary>

1. Install **Webots R2025a** for windows from [**here**](https://github.com/cyberbotics/webots/releases/download/R2025a/webots-R2025a_setup.exe)

2. Run this command in **PowerShell as Administrator**:

    ```ps
    wsl --install -d Ubuntu-22.04
    ```

3. After completing the WSL2 installation and finishing the user setup. Follow the next steps **from your WSL2 terminal session**.  

</details>

### 2. General Prerequisites

- **Webots:** Install version **R2025a** using apt from this [guide](https://cyberbotics.com/doc/guide/installation-procedure#installing-the-debian-package-with-the-advanced-packaging-tool-apt).  
  > [!NOTE]  
  > **For Windows 10/11 users:** You may notice that Webots is installed both on Windows and within WSL.  
  > This is intentional - both installations are required for proper functionality.  
- **ROS 2:** Follow the ROS 2 **Humble** [installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
- **webots_ros2:** Used to provides an interface between ROS 2 and Webots. Install from [here](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html#tasks).
- **Python3.10:** Use your preferred installation method, or follow this [tutorial](https://tutorpython.com/install-python-3-10-on-ubuntu-22-04#Prerequisites_for_installing_Python_310_on_Ubuntu_2204).

### 3. Setup Instructions

1. Clone the Repository:

    ```bash
    git clone git@github.com:MoradSayed/Advanced-Driver-Assistance-Systems.git  
    cd Advanced-Driver-Assistance-Systems/
    ```

2. Install required python packages:

    ```bash
    pip3.10 install -r requirements.txt
    ```

3. Add Environment Variable Exports to Your Shell Configuration

    Run the following command to add the necessary environment variable exports to your shell configuration file (e.g., `~/.bashrc`, `~/.zshrc`, etc.), depending on your operating system:

    <details>
    <summary><b>For Ubuntu 22.04</b></summary>

    ```bash
    cat << EOF >> ~/.bashrc

    # ADAS repo related environment variables
    export WEBOTS_HOME=/usr/local/webots
    export PYTHONPATH=\$PYTHONPATH:\$WEBOTS_HOME/lib/controller/python
    export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:\$WEBOTS_HOME/lib/controller
    export WEBOTS_PROTOCOL=ipc
    export GP_WS_DIR=$(pwd)
    EOF
    ```
    After running the above command, apply the changes by running:

    ```bash
    source ~/.bashrc
    ```
    </details>  

    <details>
    <summary><b>For Windows 10/11 (Using WSL2)</b></summary>

    ```bash
    cat << EOF >> ~/.bashrc

    # ADAS repo related environment variables
    export WEBOTS_HOME=/usr/local/webots
    export PYTHONPATH=\$PYTHONPATH:\$WEBOTS_HOME/lib/controller/python
    export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:\$WEBOTS_HOME/lib/controller
    export WINDOWS_IP=$(ip route | grep default | awk '{print $3}')
    export WEBOTS_PROTOCOL=tcp
    export GP_WS_DIR=$(pwd)
    EOF
    ```
    Apply the changes by running:

    ```bash
    source ~/.bashrc
    ```
    </details>
<!--     <br> -->

4. Follow [Webots World](/WBs/README.md#how-to-run) Instructions.

5. Follow [ROS2 WorkSpace](/Grad_ws/README.md#how-to-run) Instructions.

## License

This project is [licensed](LICENSE) under the [BSD 3-Clause License](https://opensource.org/license/bsd-3-clause).
