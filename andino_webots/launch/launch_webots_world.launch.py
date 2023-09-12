import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    andino_webots_pkg_dir = get_package_share_directory("andino_webots")
    world = LaunchConfiguration("world")
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="andino_webots.wbt",
        description="Choose one of the world files from `/andino_webots/worlds` directory",
    )
    webots = WebotsLauncher(
        world=PathJoinSubstitution([andino_webots_pkg_dir, "worlds", world]),
        ros2_supervisor=True,
    )

    return LaunchDescription(
        [
            world_arg,
            webots,
            webots._supervisor,
            # This action will kill all nodes once the Webots simulation has exited
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    )
