import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_desc = get_package_share_directory("cart_pole_description")
    pkg_bringup = get_package_share_directory("cart_pole_bringup")

    world_file = os.path.join(pkg_bringup, "worlds", "empty.sdf")
    xacro_file = os.path.join(pkg_desc, "urdf", "cart_pole.urdf.xacro")

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]),
        value_type=str
    )

    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=pkg_desc
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_args": f"-r {world_file}"
        }.items()
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "cart_pole",
            "-topic", "robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.2"
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )

    cart_effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["cart_effort_controller"],
        output="screen"
    )

    controller_node = Node(
        package="cart_pole_commander",
        executable="cart_pole_controller",
        output="screen"
    )

    logger_node = Node(
        package="cart_pole_commander",
        executable="cart_pole_logger",
        output="screen"
    )

    csv_logger_node = Node(
        package="cart_pole_commander",
        executable="cart_pole_csv",
        output="screen"
    )

    return LaunchDescription([
        gz_resource_path,
        robot_state_publisher,
        gazebo,
        spawn_entity,

        TimerAction(
            period=3.0,
            actions=[joint_state_broadcaster_spawner]
        ),

        TimerAction(
            period=5.0,
            actions=[cart_effort_controller_spawner]
        ),

        TimerAction(
            period=7.0,
            actions=[controller_node]
        ),

        TimerAction(
            period=7.5,
            actions=[logger_node , csv_logger_node]
        ),
    ])
