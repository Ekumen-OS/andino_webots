# BSD 3-Clause License

# Copyright (c) 2023, Ekumen Inc.
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import os
import launch

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import Node
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import (
    WaitForControllerConnection,
)


# Obtain andino webots package
andino_webots_pkg_dir = get_package_share_directory("andino_webots")
andino_control_pkg_dir = get_package_share_directory("andino_control")


def generate_launch_description():
    # Includes andino_description launch file
    include_webots_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                andino_webots_pkg_dir, "launch", "launch_webots_world.launch.py"
            ),
        ),
        # Define what world will be spawning
        launch_arguments={
            "world": "demo_con_motores.wbt",
        }.items(),
    )

    # Webots Controller to initialize cameras/LIDARs
    andino_webots_path = os.path.join(
        andino_webots_pkg_dir, "urdf", "andino_webots.urdf"
    )
    ros2_control_params = os.path.join(
        andino_control_pkg_dir, "config", "andino_controllers.yaml"
    )
    mappings = [
        ("/diff_controller/cmd_vel_unstamped", "/cmd_vel"),
        ("/diff_controller/odom", "/odom"),
    ]

    andino_webots_controller = WebotsController(
        robot_name="andino",
        parameters=[
            {
                "robot_description": andino_webots_path,
                "use_sim_time": False,
            },
            ros2_control_params,
        ],
        remappings=mappings,
        respawn=True,
    )

    # ROS2 control
    controller_manager_timeout = ["--controller-manager-timeout", "50"]
    diffdrive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["diff_controller"] + controller_manager_timeout,
        parameters=[
            {"use_sim_time": False},
        ],
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster"] + controller_manager_timeout,
    )
    ros_control_spawners = [
        diffdrive_controller_spawner,
        joint_state_broadcaster_spawner,
    ]

    # Wait for the simulation to be ready to start the diff drive and spawners
    waiting_nodes = WaitForControllerConnection(
        target_driver=andino_webots_controller, nodes_to_start=ros_control_spawners
    )

    # Standard ROS 2 launch description
    return launch.LaunchDescription(
        [
            include_webots_world,
            andino_webots_controller,
            waiting_nodes,
        ]
    )
