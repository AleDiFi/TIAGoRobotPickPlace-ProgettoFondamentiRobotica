from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

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
        ),
        ExecuteProcess( cmd = ['ros2','action','send_goal', '/torso_controller/follow_joint_trajectory','control_msgs/action/FollowJointTrajectory','"trajectory: {joint_names: ["torso_lift_joint"], points: [{positions: [0.35], time_from_start: {sec:10, nanosec: 0}}]}"'
                              ],
                              shell=True
                              ),
        ExecuteProcess( cmd= ['ros2','action','send_goal', '/arm_controller/follow_joint_trajectory','control_msgs/action/FollowJointTrajectory','"trajectory: {joint_names: ["arm_1_joint", "arm_2_joint", "arm_3_joint" , "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"], points: [{positions: [0.07, 0.1, -3.1, 1.36, 2.05, 0.01, -0.05], time_from_start: {sec:10, nanosec: 0}}]}"'
                              ],
                              shell=True
                              )
    ])