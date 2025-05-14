from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    # Nodo per la rilevazione degli Aruco marker
    aruco_detection_node = Node(
        package='tiago_control',
        executable='aruco_detection',
        # name='ArucoPoseEstimator',
    )

    # Nodo per la scansione della testa del Tiago
    head_scan_node = Node(
        package='tiago_control',
        executable='tiago_head_scan',
        # name='TiagoHeadScanPublisher',
    )

    # Comando per inviare la traiettoria al torso
    send_torso_trajectory = TimerAction(
        period = 13.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'action', 'send_goal',
                    '/torso_controller/follow_joint_trajectory',
                    'control_msgs/action/FollowJointTrajectory',
                    '{"trajectory": {"joint_names": ["torso_lift_joint"], "points": [{"positions": [0.35], "time_from_start": {"sec": 3}}]}}'
                ]
            )
        ]
    )

    # Comando per inviare la traiettoria al braccio, dopo 5 secondi
    send_arm_trajectory = TimerAction(
        period=17.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'action', 'send_goal',
                    '/arm_controller/follow_joint_trajectory',
                    'control_msgs/action/FollowJointTrajectory',
                    '{"trajectory": {"joint_names": ["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"], "points": [{"positions": [0.07, 0.1, -3.1, 1.36, 2.05, 0.01, -0.05], "time_from_start": {"sec": 3}}]}}'
                ]
            )
        ]
    )

    # Lista di tutte le azioni
    ld = LaunchDescription()
    ld.add_action(aruco_detection_node)
    ld.add_action(head_scan_node)
    ld.add_action(send_torso_trajectory)
    ld.add_action(send_arm_trajectory)

    return ld
