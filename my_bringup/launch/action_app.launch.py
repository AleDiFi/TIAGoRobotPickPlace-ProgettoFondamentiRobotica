from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='tiago_control',
            executable='aruco_detection',
            #name='ArucoPoseEstimator',
        ),
        Node(
            package='tiago_control',
            executable='tiago_head_scan',
            #name='TiagoHeadScanPublisher',
        )
    ])