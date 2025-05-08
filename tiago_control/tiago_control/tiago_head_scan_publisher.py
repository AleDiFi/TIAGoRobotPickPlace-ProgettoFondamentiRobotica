import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TiagoHeadScanPublisher(Node):
    def __init__(self):
        super().__init__('tiago_head_scan_client')

        self.publisher_ = self.create_publisher(JointTrajectory, "/head_controller/joint_trajectory", 10)
        self.direction = 1 
        self.completed = False
        self.timer = self.create_timer(2, self.send_trajectory)
        

    def send_trajectory(self):
        traj = JointTrajectory()
        traj.joint_names = ['head_1_joint', 'head_2_joint']

        point = JointTrajectoryPoint()
        
        if self.completed:
            head_1_value = 0.0
            head_2_value = -0.57
            point.positions = [head_1_value, head_2_value]
            point.time_from_start.sec = 2
            traj.points.append(point)
            self.publisher_.publish(traj)
            self.get_logger().info("Scan completato")
            return
        
        head_1_value = self.direction * 0.217 
        head_2_value = -0.57

        point.positions = [head_1_value, head_2_value]
        point.time_from_start.sec = 2

        traj.points.append(point)
        self.publisher_.publish(traj)

        self.get_logger().info(f"Comandata testa verso {head_1_value:.3f} rad")
        self.direction *= -1
        if head_1_value == -0.217:
            self.completed = True



def main(args=None):
    rclpy.init(args=args)
    node = TiagoHeadScanPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
