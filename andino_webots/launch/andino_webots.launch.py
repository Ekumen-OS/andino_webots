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

import launch
import os
import pathlib
import xacro

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from webots_ros2_driver.urdf_spawner import URDFSpawner
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def configure_gazebo_sensors(robot_description: str):
    """
    Configures the lidar's near parameter and attaches the camera to a rotated frame so that
    """
    robot_description = robot_description.replace("</ray>", """
                    <clip>
                        <near>0.05</near>
                    </clip>
                </ray>
    """)
    robot_description = robot_description.replace('<gazebo reference="camera_link">', '<gazebo reference="webots_camera_link">')
    return robot_description

# Obtain andino webots package
andino_webots_pkg_dir = get_package_share_directory('andino_webots')
andino_control_pkg_dir = get_package_share_directory('andino_control')
def generate_launch_description():

    andino_gazebo_xacro_path = os.path.join(andino_webots_pkg_dir, 'urdf', 'andino_webots_description.urdf.xacro')
    andino_gazebo_description = xacro.process_file(andino_gazebo_xacro_path, mappings={'use_gazebo_ros_control': 'False', 'use_fixed_caster': "False"}).toprettyxml(indent='    ')
    andino_gazebo_description = configure_gazebo_sensors(andino_gazebo_description)

    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True) # Use the /clock topic to synchronize the ROS controller with the simulation
    use_rsp = DeclareLaunchArgument('rsp', default_value='true')
    world_argument = DeclareLaunchArgument(
        'world',
        default_value='andino_webots.wbt',
    )
    # The WebotsLauncher is used to start a Webots instance.
    # Arguments:
    # - `world` (str):              Path to the world to launch.
    # - `gui` (bool):               Whether to display GUI or not.
    # - `mode` (str):               Can be `pause`, `realtime`, or `fast`.
    # - `ros2_supervisor` (bool):   Spawn the `Ros2Supervisor` custom node that communicates with a Supervisor robot in the simulation.
    webots = WebotsLauncher(
        world=PathJoinSubstitution([andino_webots_pkg_dir, 'worlds', world]),
        ros2_supervisor=True
    )

    params = {'robot_description': andino_gazebo_description,
            'publish_frequency': 30.0}

    # Robot state publisher
    rsp = Node(package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace='',
                output='both',
                parameters=[params],
                condition=IfCondition(LaunchConfiguration('rsp'))
    ) 

    # webots_ros2 node to spawn robots from URDF
    # TODO(#12): Update to PROTOSpawner when implementation is released
    spawn_andino = URDFSpawner(
        name='andino',
        robot_description=andino_gazebo_description,
        translation='0 0 0.022',
        rotation=' 0 0 1 0',
    )
    # Webots Controller to initialize cameras/LIDARs
    andino_webots_path = os.path.join(andino_webots_pkg_dir, 'urdf', 'andino_webots.urdf')
    ros2_control_params = os.path.join(andino_control_pkg_dir, 'config', 'andino_controllers.yaml')
    mappings = [('/diff_controller/cmd_vel_unstamped', '/cmd_vel'), ('/diff_controller/odom', '/odom')]
    andino_webots_controller = WebotsController(
        robot_name='andino',
        parameters=[
            {'robot_description': andino_webots_path,
             'use_sim_time': use_sim_time,
            },
            ros2_control_params
        ],
        remappings=mappings,
        respawn=True
    )

    # ROS2 control
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['diff_controller'] + controller_manager_timeout,
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    ros_control_spawners = [diffdrive_controller_spawner, joint_state_broadcaster_spawner]

    # Wait for the simulation to be ready to start the diff drive and spawners
    waiting_nodes = WaitForControllerConnection(
        target_driver=andino_webots_controller,
        nodes_to_start= ros_control_spawners
    )

    # Standard ROS 2 launch description
    return launch.LaunchDescription([
        # Set the world argument
        world_argument,
        # Start the Webots node
        webots,
        # Starts the Ros2Supervisor node created with the WebotsLauncher
        webots._supervisor,
        # Spawn Andino's URDF
        spawn_andino,
        # Add andino's controller
        andino_webots_controller,
        waiting_nodes,
        # Robot state publisher
        use_rsp,
        rsp,
        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
