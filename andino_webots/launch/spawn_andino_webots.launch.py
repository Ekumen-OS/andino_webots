import os
import pathlib
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

andino_webots_pkg_dir = get_package_share_directory('andino_webots')
andino_control_pkg_dir = get_package_share_directory('andino_control')
def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default=True) # Use the /clock topic to synchronize the ROS controller with the simulation
    use_rsp = DeclareLaunchArgument('rsp', default_value='true')

    andino_gazebo_xacro_path = os.path.join(andino_webots_pkg_dir, 'urdf', 'andino_webots_description.urdf.xacro')
    andino_gazebo_description = xacro.process_file(andino_gazebo_xacro_path, mappings={'use_gazebo_ros_control': 'False', 'use_fixed_caster': "False"}).toprettyxml(indent='    ')
    andino_gazebo_description = configure_gazebo_sensors(andino_gazebo_description)

    # TODO(#12): Update to PROTOSpawner when implementation is released
    spawn_andino = URDFSpawner(
        name='andino',
        robot_description=andino_gazebo_description,
        translation='0 0 0.022',
        rotation=' 0 0 1 0',
    )

    # Robot state publisher
    params = {'robot_description': andino_gazebo_description,
            'publish_frequency': 30.0}
    rsp = Node(package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace='',
                output='both',
                parameters=[params],
                condition=IfCondition(LaunchConfiguration('rsp'))
    ) 

    andino_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': 'Andino Webots'},
        parameters=[
            {'robot_description': andino_gazebo_description},
        ],
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

    return LaunchDescription([
        # Request to spawn the Andino via URDF
        spawn_andino,
        # Add andino's controller
        andino_webots_controller,
        waiting_nodes,
        # Robot state publisher
        use_rsp,
        rsp,        
        # Launch the driver node once the URDF robot is spawned
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessIO(
                target_action=spawn_andino,
                on_stdout=lambda event: get_webots_driver_node(event, andino_robot_driver),
            )
        ),
        # Kill all the nodes when the driver node is shut down (useful with other ROS 2 nodes)
        launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=andino_robot_driver,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
    ])
