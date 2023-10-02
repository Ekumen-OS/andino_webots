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

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Obtain andino webots package
andino_webots_pkg_dir = get_package_share_directory("andino_webots")


def generate_launch_description():
    world = LaunchConfiguration("world")
    rsp = LaunchConfiguration("rsp")
    use_sim_time = LaunchConfiguration("use_sim_time")

    remove_nodes_arg = DeclareLaunchArgument(
        "remove_nodes",
        default_value="true",
        description="Enable NodeRemover robot spawning.",
    )
    use_rsp = DeclareLaunchArgument(
        "rsp",
        default_value="true",
        description="Select whether to spawn the Robot State Publisher",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use the /clock topic to synchronize the ROS controller with the simulation",
    )
    world_argument = DeclareLaunchArgument(
        "world",
        default_value="room.wbt",
        description="Select world to spawn. Must be present in andino_webots/worlds",
    )

    # Includes andino_description launch file
    include_webots_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                andino_webots_pkg_dir, "launch", "launch_webots_world.launch.py"
            ),
        ),
        # Define what world will be spawning
        launch_arguments={
            "world": world,
        }.items(),
    )

    include_spawn_andino = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                andino_webots_pkg_dir, "launch", "spawn_andino_webots.launch.py"
            ),
        ),
        launch_arguments={
            "rsp": rsp,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # Include node remover supervisor plugin launch file
    include_supervisor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(andino_webots_pkg_dir, "launch", "remove_nodes.launch.py"),
        ),
        # Define what world will be spawning
        launch_arguments={
            "remove_nodes": LaunchConfiguration("remove_nodes"),
        }.items(),
    )

    # Standard ROS 2 launch description
    return launch.LaunchDescription(
        [
            remove_nodes_arg,
            use_rsp,
            use_sim_time_arg,
            world_argument,
            include_webots_world,
            include_spawn_andino,
            include_supervisor,
        ]
    )
