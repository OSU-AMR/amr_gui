from launch.actions import GroupAction, DeclareLaunchArgument, Shutdown
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration as LC
from launch_ros.actions import Node, PushRosNamespace
from launch import LaunchDescription

from ament_index_python.packages import get_package_share_directory

import os

robot = 'central'

rviz_pkg_dir = get_package_share_directory('amrviz')

rviz_src = os.path.join(rviz_pkg_dir[: rviz_pkg_dir.find("install") - 1], "src", "amr_gui", "amrviz", "amrviz")

def generate_launch_description():
    return LaunchDescription([
        GroupAction(
            actions=[
                # declare the launch args here
                # DeclareLaunchArgument(
                #     "robot", 
                #     default_value=robot,
                #     description="Name of the vehicle",
                # ),

                DeclareLaunchArgument(
                    "control_config_file",
                    default_value=["control_config_", "central", ".rviz"]
                ),

                # DeclareLaunchArgument('robot_yaml', default_value=[LC("robot"), '.yaml']),

                # start rviz
                Node(
                    package='rviz2',
                    executable='rviz2',
                    # DONT USE IT RENAMES ALL OF THE CHILD NODES CREATED FOR THE PLUGINS
                    # name='riptide_rviz', 
                    on_exit=Shutdown(),
                    arguments=[
                        "-d", 
                        PathJoinSubstitution([
                            rviz_src,
                            LC("control_config_file")
                        ])
                    ],
                    # prefix=["gdbserver localhost:3000"]
                ),

                # send the rest into the robot namespace
                PushRosNamespace(["/",  "central"]),
                
                Node(
                    package="amrviz",
                    executable="PosePublisher.py",
                    name="gridpublisher",
                    output="screen",
                    parameters = [
                    ]
                )
            ]
        )
    ])

