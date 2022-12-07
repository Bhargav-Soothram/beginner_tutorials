from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Arguments to the executable
    msg_arg = DeclareLaunchArgument('my_message', default_value = TextSubstitution(text="Stranger Things!"), description="Text for the publisher to publish")
    freq_arg = DeclareLaunchArgument('my_message_freq', default_value = TextSubstitution(text="1000"), description="Time between successive messages")
    ros_bag_arg = DeclareLaunchArgument('rosbag_record', default_value = TextSubstitution(text = "False"), choices = ['True', 'False'], description = "Bool for switching ros bag recording on/off")

    publisher = Node(
            package='cpp_pubsub',
            executable='talker',
            parameters=[
                {"my_message" : LaunchConfiguration('my_message')},
                {"my_message_freq" : LaunchConfiguration('my_message_freq')}
            ]
        )

    subscriber = Node(
            package = 'cpp_pubsub',
            executable = 'listener',
        ) 
    
    recorder = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('rosbag_record')),
            cmd=['ros2', 'bag', 'record', '-a'],
        shell=True
    )

    return LaunchDescription([
        msg_arg,
        freq_arg,
        ros_bag_arg,
        publisher,
        subscriber,
        recorder
    ])