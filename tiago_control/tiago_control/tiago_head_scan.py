import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class TiagoHeadScanActionClient(Node):

    "client per il movimento della testa di Tiago in scan"

    def __init__(self):
        super().__init__('tiago_head_scan_action_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory') 
        self.trajectory_index = 0
        self.trajectories = self.create_trajectories()
        self.send_next_goal()

    def create_trajectories(self):
        """Crea le due traiettorie per eseguire lo scan della testa."""
        joint_names = ['head_1_joint', 'head_2_joint']
        head_positions = [
            [ 0.217, -0.57],  # prima direzione
            [-0.217, -0.57],  # seconda direzione (opposta)
            [ 0.0, -0.57],  # posizione centrale
            [ 0.217, -0.57],  # ritorno alla prima posizione
            [-0.217, -0.57],  # ritorno alla seconda posizione
            [ 0.0, -0.57]   # ritorno alla posizione centrale
        ]

        trajectories = []

        for pos in head_positions:
            """Genera un messaggio di traiettoria per ogni posizione della testa."""
            goal_msg = FollowJointTrajectory.Goal() #goal message
            goal_msg.trajectory.joint_names = joint_names #messaggio di traiettoria

            point = JointTrajectoryPoint() #messaggio di punto della traiettoria
            point.positions = pos #posizione della testa 
            point.time_from_start.sec = 2

            goal_msg.trajectory.points.append(point)
            trajectories.append(goal_msg)

        return trajectories

    def send_next_goal(self):
        """Invia il  goal alla testa di Tiago.
        Se sono state inviate tutte le traiettorie, termina l'azione."""
        if self.trajectory_index >= len(self.trajectories): #se sono state inviate  le traiettorie
            self.get_logger().info('Scan completato.') 
            return
        

        goal_msg = self.trajectories[self.trajectory_index] 
        self._action_client.wait_for_server() #aspetta il server
        self.get_logger().info(f'Invio traiettoria {self.trajectory_index + 1}...') 
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback) 
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):

        """Callback per la risposta del goal
            input: future. future è un oggetto che rappresenta il risultato di un'operazione asincrona
            output: goal_handle"""
        goal_handle = future.result()
        if not goal_handle.accepted:

            #se il goal non è accettato
            self.get_logger().warning('Goal rifiutato.')
            return

        self.get_logger().info('Goal accettato. \n R0G3R-----R0G3R') 
        self._get_result_future = goal_handle.get_result_async() # ottieni il risultato
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):

        """Callback per il risultato del goal.
            input: future. future è un oggetto che rappresenta il risultato di un'operazione asincrona
            output: result"""
        result = future.result().result 
        self.get_logger().info(f'Traiettoria {self.trajectory_index + 1} completata.')
        self.trajectory_index += 1
        self.send_next_goal()

    def feedback_callback(self, feedback_msg):
        """Callback per ricevere il feedback della traiettoria.
            input: feedback_msg. feedback_msg è un oggetto che rappresenta il messaggio di feedback
            output: feedback_msg"""
        self.get_logger().info(f'Feedback ricevuto per traiettoria {self.trajectory_index + 1}.')
        desired = feedback_msg.feedback.desired.positions
        actual = feedback_msg.feedback.actual.positions
        error = feedback_msg.feedback.error.positions

        for i, (d, a, e) in enumerate(zip(desired, actual, error)):
            self.get_logger().info(
                f"[Giunto {i}] Desired: {d:.3f}, Actual: {a:.3f}, Error: {e:.3f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = TiagoHeadScanActionClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
