# Andino Webots Simulation

<!-- Image of andino in webots with shadows and stuff -->
<!-- Description and purpose -->
![ANDINO WEBOTS SIMULATION](image.png)
## Description

This package provides a simulation environment for Andino in Webots using webots_ros2 <!-- Hyperlink to the package here -->, exposing its sensors readings and enabling commanding the robot using `/cmd_vel`. The robot will be spawned in a new world file present in the package.

### Webots_ros2 package
TODO
Webots_ros2 provides a simple 

### urdf2webots
TODO
Since Webots uses `PROTO` as their robot description format, and the Andino is written in `URDF`. The `urdf2webots` package, a tool to convert urdf to a Webots-compatible proto file, and which is already present within the `webots_ros2` package, is used.
Notably, the tool lacks functionality for several  

## Installation

This package makes use of some packages from https://github.com/Ekumen-OS/andino repository. Therefore, the repository is brought as a git submodule.
For so, when cloning this repository make sure to also init the submodules, this can be done adding a `--recursive` flag to the `git clone` command

1. Clone this repository

```sh
git clone git@github.com:ekumenlabs/andino_webots.git --recursive
```

2. Set up docker environment:
Refer to [docker readme](docker/README.md)

Once the container is running and dependencies have been installed you can proceed to package building.

## Build

In order to run the simulation you must first build the package and then source the current terminal to the install workspace. To do so run the following commands:

```sh
colcon build
source install/setup.bash
```

## Usage

### Start simulation
Once the package is built and the terminal sourced, you can start the simulation by launching the a ROS2 launchfile, which starts an empty Webots simulation:


```sh
ros2 launch andino_webots launch_webots_world.launch.py
```

### Spawn andino in a webots_ros2 simulation

To add an Andino to the simulation you may run a separate launchfile that spawns an Andino in an already-running webots_ros2 simulation:


```sh
ros2 launch andino_webots spawn_andino_webots.launch.py
```

Alternatively you can launch the simulation along with the robot by running the `andino_webots.launch.py` launchfile, which includes both previous commands:

```sh
ros2 launch andino_webots andino_webots.launch.py
```

### Teleoperate Andino in Webots

The robot is connected to the `webots_ros2_control` plugin, that enables teleoperation by requesting commands via `/cmd_vel`; so, out of the box, the package is ready to accept velocity commands.

Furthermore, the package allows sensor readings to be exposed to the ROS2 network, in the topics listed on the `andino_webots.urdf` description file.

### Rviz

TODO  
An Rviz configuration is provided to easily show the sensors outputs


<!--

Examples

:tick: How to build and install (should reference external tools/workflows)

TOOD: How to develop (useful for describing things like python setup.py develop)

TODO: License and copyright statements
 -->
