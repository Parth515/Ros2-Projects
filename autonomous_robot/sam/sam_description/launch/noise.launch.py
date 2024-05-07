from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def noisy_controller(context, *args, **kwargs):
    wheel_radius = float(LaunchConfiguration("wheel_radius").perform(context))
    wheel_separation = float(LaunchConfiguration("wheel_separation").perform(context))
    wheel_radius_error = float(LaunchConfiguration("wheel_radius_error").perform(context))
    wheel_separation_error = float(LaunchConfiguration("wheel_separation_error").perform(context))

    noisy_controller_py = Node(
        package="sam_description",
        executable="noisy_controller.py",
        parameters=[
            {"wheel_radius": wheel_radius + wheel_radius_error,
             "wheel_separation": wheel_separation + wheel_separation_error}],
    )

    return [
        noisy_controller_py,
    ]



def generate_launch_description():
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.1",
    )
    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.025",
    )
    wheel_radius_error_arg = DeclareLaunchArgument(
        "wheel_radius_error",
        default_value="0.005",
    )
    wheel_separation_error_arg = DeclareLaunchArgument(
        "wheel_separation_error",
        default_value="0.001",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["sam_description", 
                   "--controller-manager", 
                   "/controller_manager"
        ],
    )

    noisy_controller_launch = OpaqueFunction(function=noisy_controller)

    return LaunchDescription(
        [
            wheel_radius_arg,
            wheel_separation_arg,
            wheel_radius_error_arg,
            wheel_separation_error_arg,
            joint_state_broadcaster_spawner,
            wheel_controller_spawner,
            noisy_controller_launch,
        ]
    )
