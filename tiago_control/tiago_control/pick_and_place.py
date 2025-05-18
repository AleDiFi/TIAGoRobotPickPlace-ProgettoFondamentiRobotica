import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
import numpy as np
from spatialmath import SE3
from scipy.spatial.transform import Rotation as R
import roboticstoolbox as rtb
from roboticstoolbox import ERobot
from sensor_msgs.msg import JointState


class TiagoPickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('tiago_pick_and_place_node')

        self.joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint',
                            'arm_5_joint', 'arm_6_joint', 'arm_7_joint']
        
        self.trajectory_generated = False
        
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        self.qtraj = self.create_subscription(PoseStamped, '/aruco_marker/ID2/transformed_pose', self.pose1_callback, 10)

        self.arm_action_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

        urdf_loc = '/home/lorenzo/tiago_public_ws/src/tiago_control/tiago_control/tiago_robot.urdf'
        self.robot = ERobot.URDF(urdf_loc)

        self.get_logger().info("Tiago Pick and Place Node Avviato")

    def joint_state_callback(self, msg):
        self.current_joint_positions = msg.position
        self.get_logger().info(f"Posizioni giunti attuali: {self.current_joint_positions[:7]}")

    def pose1_callback(self, msg):
        if self.trajectory_generated:
            return  # Traiettoria già calcolata, ignora nuovi messaggi

        if not hasattr(self, 'current_joint_positions'):
            self.get_logger().warning("Joint states not received yet.")
            return

        q0 = np.array(self.current_joint_positions[:7], dtype=float)

        try:
            T0 = self.robot.fkine(q0)
        except Exception as e:
            self.get_logger().error(f"Errore nel calcolo della fkine: {e}")
            return

        position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z - 0.1  # correzione z fissa
        ])
        quaternion = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]

        try:
            rotation = R.from_quat(quaternion).as_matrix()
            TF = SE3.Rt(rotation, position)
        except Exception as e:
            self.get_logger().error(f"Errore nella costruzione della trasformazione finale TF: {e}")
            return

        N = 20
        try:
            trajectory = rtb.ctraj(T0, TF, N)
        except Exception as e:
            self.get_logger().error(f"Errore nella generazione della traiettoria: {e}")
            return

        q_traj = []
        q_curr = q0
        for T in trajectory:
            sol = self.robot.ik_NR(T, q0=q_curr, pinv=True)
            if sol is not None and len(sol) > 0:
                q_curr = sol[0]
                q_traj.append(q_curr)
            else:
                self.get_logger().warning(f"IK fallita in un punto della traiettoria.")
                return

        q_traj = np.array(q_traj)
        q_traj_braccio = q_traj[:, :7]

        # Blocca ulteriori elaborazioni
        self.trajectory_generated = True

        self.prepare_trajectory(q_traj_braccio)

    def prepare_trajectory(self, q_traj_braccio):
        self.trajectory_index = 0
        self.trajectories_msgs = []

        for q in q_traj_braccio:
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory.joint_names = self.joint_names

            point = JointTrajectoryPoint()
            point.positions = [float(p) for p in q]
            point.time_from_start.sec = 2

            goal_msg.trajectory.points.append(point)
            self.trajectories_msgs.append(goal_msg)

        self.send_next_goal()

    def send_next_goal(self):
        if self.trajectory_index >= len(self.trajectories_msgs):
            self.get_logger().info('Traiettoria COMPLETATA con successo.')
            return

        goal_msg = self.trajectories_msgs[self.trajectory_index]
        self.arm_action_client.wait_for_server()
        self.get_logger().info(f'Invio punto {self.trajectory_index + 1} della traiettoria...')
        self._send_goal_future = self.arm_action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Goal rifiutato.')
            return

        self.get_logger().info(f'Goal accettato [{self.trajectory_index + 1}].')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Punto {self.trajectory_index + 1} completato.')
        self.trajectory_index += 1
        self.send_next_goal()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback ricevuto per punto {self.trajectory_index + 1}.')
        desired = feedback_msg.feedback.desired.positions
        actual = feedback_msg.feedback.actual.positions
        error = feedback_msg.feedback.error.positions

        for i, (d, a, e) in enumerate(zip(desired, actual, error)):
            self.get_logger().info(
                f"[Giunto {i}] Desired: {d:.3f}, Actual: {a:.3f}, Error: {e:.3f}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = TiagoPickAndPlaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
