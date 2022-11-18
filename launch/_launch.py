from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    msg_arg = DeclareLaunchArgument('my_message', default_value = TextSubstitution(text="Stranger Things!"))
    freq_arg = DeclareLaunchArgument('my_message_freq', default_value = TextSubstitution(text="1000"))

    return LaunchDescription([
        msg_arg,
        freq_arg,
        Node(
            package='cpp_pubsub',
            executable='talker',
            parameters=[
                {"my_message" : LaunchConfiguration('my_message')},
                {"my_message_freq" : LaunchConfiguration('my_message_freq')}
            ]
        ),
        Node(
            package='cpp_pubsub',
            executable='listener'
        )
    ])