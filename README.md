## NOVAMOB


This repository provides simulation environments for the **NovaMob** and **Agilex Scout** robots in **ROS2**, compatible with both **Gazebo Classic** (only basic features) and **Gazebo Fortress** (Ignition Gazebo). This packages include different robot models, sensors, controllers, and configuration files necessary for seamless simulation and testing.


## Installation

### Prerequisites

- **ROS2** Humble installed on your system.
- **Gazebo Classic** and/or **Gazebo Fortress** installed.

### Clone the Repository

1. Create or navigate to your ROS2 workspace:

    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```

2. Clone this repository and its submodules into your workspace:

    ```bash
    git clone --recurse-submodules https://github.com/TomasPedreira/tese_ws.git
    ```
3. Install necessary dependencies:

    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

4. Build the workspace:

    ```bash
    colcon build
    ```
5. Source the workspace:

    ```bash
    source ~/ros2_ws/install/setup.bash
    ```
    
## Robot Descriptions

### Agilex Scout
The Scout is a bigger 4 wheel drive custom robot more suited for outdoor environments with a trailer.

It is equipped with the following sensors:
- **VLP16 LIDAR**
- **IMU mpu6050**
- **Intel Realsense D435if**
- **GPS sensor**

### Novamob
Novamob is a small 4 wheel drive custom robot designed for indoor environments. This is not the focus of this thesis ad is a result of a previous project developed in the original repo.

## Launch files

### Visualizing the robots

To visualize the Agilex Scout robot in Gazebo Fortress, use the following command:
 
```bash
ros2 launch scout_nav2_gz display.launch.py use_trailer:=True
```

To visualize the Agilex Scout robot in Gazebo Classic, use the following command:
 
```bash
ros2 launch scout_description classic_display.launch.py
```

### Running the Navigation Stack

To run the navigation stack for the Agilex Scout robot (when using Fortress, Classic has navigation integrated in the launch file), use the following command:

```bash
ros2 launch scout_nav2_gz complete_navigation.launch.py use_trailer:=True
```

## Useful Resources

- [Nav2 Documentation](https://docs.nav2.org/concepts/index.html)
