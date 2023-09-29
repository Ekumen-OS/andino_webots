# Andino Webots Simulation :lady_beetle:

<!-- Image of andino in webots with shadows and stuff -->
<!-- Description and purpose -->
![ANDINO WEBOTS SIMULATION](andino_webots/docs/andino_sim.png)

## Description :clipboard:

This package provides a simulation environment for Andino in Webots using [webots_ros2](https://github.com/cyberbotics/webots_ros2) to integrate it with ROS2.

Webots uses [PROTO](https://cyberbotics.com/doc/reference/proto) as their robot description format, and the Andino is written in `URDF`. The `urdf2webots` package provides a tool to convert a robot description to a Webots-compatible proto file. This tool, which is already present within the `webots_ros2` package, is used to convert the robot description in runtime.
<!-- 
### urdf2webots
TODO
Since Webots uses `PROTO` as their robot description format, and the Andino is written in `URDF`. The `urdf2webots` package, a tool to convert urdf to a Webots-compatible proto file, and which is already present within the `webots_ros2` package, is used.
Notably, the tool lacks functionality for several   -->

## Installation :inbox_tray:


This package makes use of some packages from https://github.com/Ekumen-OS/andino repository. Therefore, the repository is brought as a git submodule.
For so, when cloning this repository make sure to also init the submodules, this can be done adding a `--recursive` flag to the `git clone` command

1. Clone this repository

```sh
git clone git@github.com:ekumenlabs/andino_webots.git --recursive
```

2. Set up docker environment:
Refer to [docker readme](docker/README.md)

Once the container is running and dependencies have been installed you can proceed to package building.

## Build :package:

The package contains some dependencies that must be installed in order to build it:

```
rosdep install --from-paths src -i -y
```

Then build the package and source the install workspace. To do so run the following commands:

```sh
colcon build
source install/setup.bash
```

## Usage :rocket:

### Start empty simulation

Once the package is built and sourced, you can start an empty simulation by running the following ROS2 launch file:


```sh
ros2 launch andino_webots launch_webots_world.launch.py world:=andino_webots
```

This launch file starts an empty simulation in a world given by the `world` argument. This argument defines the wbt file of the world where andino will run. It should be present in the package's `world` folder.   
Defaults to: 'andino_webots'.
 

### Spawn andino in a webots_ros2 simulation

To spawn an Andino to a running webots_ros2 simulation you may run a separate launchfile:


```sh
ros2 launch andino_webots spawn_andino_webots.launch.py
```

This launch file supports the following launch arguments:

- `use_sim_time` . Parameter to indicate to the robot controller to synchronize with simulation time. Defaults to 'true'.
- `rsp` . Parameter to decide whether to spawn the Robot State Publisher node or not. Defaults to 'true'.

### Start an Andino Webots simulation

Alternatively you can launch the simulation along with the robot by running the `andino_webots.launch.py` launchfile, which includes both previous commands:

```sh
ros2 launch andino_webots andino_webots.launch.py
```

This launchfile accepts all previous arguments, with the addition of the choice to run a custom plugin to remove nodes from a robot in the simulation:

- `remove_nodes` . Decide whether to run the `NodeRemover` plugin, which removes specific nodes from a robot in the simulation. The parameters for this plugin can be set in the `node_remover_plugin.urdf` file in the package. Defaults to 'true'.

### Teleoperate Andino in Webots :joystick:

The robot is connected to the `webots_ros2_control` plugin, that enables teleoperation by requesting commands via `/cmd_vel`; so, out of the box, the package is ready to accept velocity commands.

Furthermore, the package allows sensor readings to be exposed to the ROS2 network, in the topics listed on the `andino_webots.urdf` description file.

![](andino_webots/docs/andino.gif)

## NodeRemover plugin :wrench:

This package also provides a custom `webots_ros2` plugin as an example of the package's capabilities, as well as a means of providing a workaround for `urdf2webots` not having a straightforward way to generate a free rotating joint.
The associated [launchfile](./andino_webots/launch/remove_nodes.launch.py) spawns a Supervisor robot in a running simulation and attaches a custom plugin to it, defined in the [node_remover_plugin](./node_remover_plugin) subpackage, which takes in a robot's name and list of the nodes to be removed as parameters.
