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
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import (
    WaitForControllerConnection,
)


def apply_colors(robot_description: str):
    """
    Function to provide a hotfix to urdf2webots' issue with color declaration.
    The package seems to require the color to be defined in a link at least once
    to be used consecutively. To solve this issue without modifying the andino description package
    the urdf is modified to define the color on each link that requires it
    See #210 *https://github.com/cyberbotics/urdf2webots/issues/210)
    """
    colors_definitions = {
        "yellow": '"1 0.95 0 1"',
        "blue": '"0 0 1 1"',
        "light_blue": '"0 0.5 0.8 1"',
        "black": '"0 0 0 1"',
        "white": '"1 1 1 1"',
        "red": '"0.8 0.0 0.0 1.0"',
        "silver": '"0.79 0.82 0.93 1"',
        "dark_grey": '"0.3 0.3 0.3 1"',
    }
    for color_name, color_values in colors_definitions.items():
        robot_description = robot_description.replace(
            f'<material name="{color_name}"/>',
            f"""
        <material>
            <color rgba={color_values}/>
        </material>""",
        )
    return robot_description


def configure_gazebo_sensors(robot_description: str):
    """
    Configures the lidar's near parameter and attaches the camera to a rotated frame so that
    """
    robot_description = robot_description.replace(
        "</ray>",
        """
                    <clip>
                        <near>0.05</near>
                    </clip>
                </ray>
        """,
    )
    robot_description = robot_description.replace(
        '<gazebo reference="camera_link">', '<gazebo reference="webots_camera_link">'
    )
    robot_description = apply_colors(robot_description)
    return robot_description


andino_webots_pkg_dir = get_package_share_directory("andino_webots")
andino_control_pkg_dir = get_package_share_directory("andino_control")


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    use_rsp = DeclareLaunchArgument(
        "rsp",
        default_value="true",
        description="Select whether to spawn the Robot State Publisher",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
    )

    andino_description_xacro_path = os.path.join(
        andino_webots_pkg_dir, "urdf", "andino_webots_description.urdf.xacro"
    )
    andino_description = xacro.process_file(
        andino_description_xacro_path,
        mappings={"use_gazebo_ros_control": "False", "use_fixed_caster": "False"},
    ).toprettyxml(indent="    ")
    andino_webots_description = configure_gazebo_sensors(andino_description)

    # TODO(#12): Update to PROTOSpawner when implementation is released
    spawn_andino = URDFSpawner(
        name="andino",
        robot_description=andino_webots_description,
        translation="0 0 0.022",
        rotation=" 0 0 1 0",
    )

    # Robot state publisher
    params = {"robot_description": andino_description, "publish_frequency": 30.0}
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="",
        output="both",
        parameters=[params],
        condition=IfCondition(LaunchConfiguration("rsp")),
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
                "use_sim_time": use_sim_time,
            },
            ros2_control_params,
        ],
        remappings=mappings,
        respawn=True,
    )

    # ROS2 control
    controller_manager_timeout = ["--controller-manager-timeout", "5000"]
    diffdrive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["diff_controller"] + controller_manager_timeout,
        parameters=[
            {"use_sim_time": use_sim_time},
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

    return LaunchDescription(
        [
            use_sim_time_arg,
            # Request to spawn the Andino via URDF
            spawn_andino,
            # Add andino's controller
            andino_webots_controller,
            waiting_nodes,
            # Robot state publisher
            use_rsp,
            rsp,
        ]
    )
