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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    andino_webots_pkg_dir = get_package_share_directory("andino_webots")

    remove_nodes = LaunchConfiguration("remove_nodes")

    remove_nodes_arg = DeclareLaunchArgument(
        "remove_nodes",
        default_value="false",
        description="Enable Supervisor robot spawning.",
    )

    # Launch supervisor
    supervisor_robot_description = """
        data: Robot { supervisor TRUE name "NodeRemover" controller "<extern>"}
        """
    command = [
        "ros2",
        "service",
        "call",
        "/Ros2Supervisor/spawn_node_from_string",
        "webots_ros2_msgs/srv/SpawnNodeFromString",
        supervisor_robot_description,
    ]
    spawn_supervisor = ExecuteProcess(
        cmd=command,
        output="log",
        condition=IfCondition(remove_nodes),
    )
    # Plugin to control supervisor
    node_remover_plugin_path = os.path.join(
        andino_webots_pkg_dir, "urdf", "node_remover_plugin.urdf"
    )
    supervisor_controller = WebotsController(
        robot_name="NodeRemover",
        parameters=[{"robot_description": node_remover_plugin_path}],
        condition=IfCondition(remove_nodes),
    )

    return LaunchDescription(
        [
            remove_nodes_arg,
            spawn_supervisor,
            supervisor_controller,
        ]
    )
