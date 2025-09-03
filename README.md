## AGX SCOUT


This repository provides simulation environments for the **NovaMob** and **Agilex Scout** robots in **ROS2**, compatible with both **Gazebo Classic** and **Gazebo Fortress** (Ignition Gazebo). This packages include different robot models, sensors, controllers, and configuration files necessary for seamless simulation and testing.

This repo is a fork from the original developer of the NovaMob and focouses on the navigation of the Agilex Scout.
The relevant directories for this matter are:
- scout_descrioption: URDF/Trailer position estimator/EKF/Navigation parameters
- pp_controller: Controller definitions
- astar_planner: Voronoi Hybrid A* custom planner inspired by https://github.com/tanujthakkar/Voronoi-Based-Hybrid-Astar.


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
- **RS_HELIOS_16P LIDAR**
- **Intel Realsense D435if**

### Novamob
Novamob is a small 4 wheel drive custom robot designed for indoor environments. This is not the focus of this thesis and is a result of a previous project developed in the original repo.

## Launch files

### Visualizing the robots

### Agilex Scout (Main focus)
To visualize the Agilex Scout robot in Gazebo Fortress (only visualization), use the following command:
 
```bash
ros2 launch scout_nav2_gz display.launch.py use_trailer:=True
```

To visualize and use the Agilex Scout robot in Gazebo Classic (Recomended), use the following command:
 
```bash
colcon build
source install/setup.bash
ros2 launch scout_description classic_display.launch.py
```

## Useful Resources

- [Nav2 Documentation](https://docs.nav2.org/concepts/index.html)
