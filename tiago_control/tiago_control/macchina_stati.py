from enum import Enum, auto
import rclpy
from rclpy.node import Node
from subprocess import Popen
import time
import signal

# Questo codice è un esempio di un automa a stati finiti (FSM) in Python per controllare il robot Tiago.
# Utilizza la libreria rclpy per interagire con ROS2 e subprocess per eseguire comandi esterni.
# La FSM ha diversi stati: START, ARUCO, HEAD_SCAN, TORSO, ARM e DONE.
# Ogni stato rappresenta una fase del processo di rilevamento e movimento del robot.
# La FSM inizia con lo stato START, dove viene avviato il nodo di rilevamento degli Aruco.
# Una volta completato il rilevamento, passa allo stato HEAD_SCAN per eseguire la scansione della testa.
# Dopo la scansione della testa, invia una traiettoria al torso e infine al braccio.
# Ogni stato esegue un'azione specifica e passa al successivo quando l'azione è completata.
# La FSM termina quando tutte le azioni sono completate e il timer viene annullato.



class State(Enum):
    START = auto()
    HEAD_SCAN = auto()
    TORSO = auto()
    ARM = auto()
    DONE = auto()

class Sequencer(Node):
    def __init__(self):
        super().__init__('sequencer_fsm')
        self.state = State.START
        self.timer = self.create_timer(1.0, self.loop)  
        self.start_time = None
        self.aruco_timeout_sec = 15

    def loop(self):
        if self.state == State.START:
            self.get_logger().info('Avvio Aruco detection e Head scan...')
            self.aruco_proc = Popen(['ros2', 'run', 'tiago_control', 'aruco_detection'])
            self.head_proc = Popen(['ros2', 'run', 'tiago_control', 'tiago_head_scan'])
            self.start_time = time.time()
            self.state = State.HEAD_SCAN
            self.get_logger().info('Aruco detection e Head scan avviati.')

        elif self.state == State.HEAD_SCAN:
            now = time.time()
            # Controlla se i processi sono terminati
            # e se il timeout è scaduto

            # aruco_done = self.aruco_proc.poll() is not None
            head_done = self.head_proc.poll() is not None
            # timeout_expired = now - self.start_time > self.aruco_timeout_sec
            self.get_logger().info(f'Head done: {head_done}')

            '''
            now: è il tempo corrente in secondi (ottenuto con time.time())
            self.start_time: è il tempo in secondi in cui è stato avviato aruco_detection
            now - self.start_time: è il tempo trascorso dall'avvio
            self.aruco_timeout_sec: è il timeout massimo che vuoi aspettare (in secondi), es. 15
            '''

            if head_done: #head_done and (aruco_done or timeout_expired):

                self.get_logger().info('Aruco + Head scan completati. Avvio torso...')
                self.torso_proc = Popen([
                    'ros2', 'action', 'send_goal',
                    '/torso_controller/follow_joint_trajectory',
                    'control_msgs/action/FollowJointTrajectory',
                    '{"trajectory": {"joint_names": ["torso_lift_joint"], "points": [{"positions": [0.35], "time_from_start": {"sec": 3}}]}}'
                ])
                self.intermediate_arm_proc = Popen(['ros2', 'action', 'send_goal',
                    '/arm_controller/follow_joint_trajectory',
                    'control_msgs/action/FollowJointTrajectory',
                    '{"trajectory": {"joint_names": ["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"], "points": [{"positions": [0.0003836728901962516, -0.0001633239063343339, -9.037018213753356e-06, -6.145563957549172e-05, 4.409014973383307e-05, 0.0019643255648595925, 0.0004167305736686444], "time_from_start": {"sec": 3}}]}}'
                ])
                self.state = State.TORSO

        elif self.state == State.TORSO:
            if self.torso_proc.poll() is not None:
                self.get_logger().info('Torso completato. Avvio braccio...')
                self.arm_proc = Popen([
                    'ros2', 'action', 'send_goal',
                    '/arm_controller/follow_joint_trajectory',
                    'control_msgs/action/FollowJointTrajectory',
                    '{"trajectory": {"joint_names": ["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"], "points": [{"positions": [0.07, 0.1, -3.1, 1.36, 2.05, 0.01, -0.05], "time_from_start": {"sec": 3}}]}}'
                ])
                self.state = State.ARM

        elif self.state == State.ARM:
            if self.arm_proc.poll() is not None:
                self.get_logger().info('Braccio completato. Sequenza terminata.')
                self.state = State.DONE

        elif self.state == State.DONE:
            self.get_logger().info('FSM completata. Arresto timer.')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = Sequencer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
