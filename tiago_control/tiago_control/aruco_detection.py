import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf_transformations

class ArucoPoseEstimator(Node):
    def __init__(self):
        super().__init__('aruco_pose_estimator')

        self.bridge = CvBridge()
        self.marker_size = 0.06 # Marker size in meters

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters()

        self.camera_matrix = None
        self.dist_coeffs = None

        self.detected_ids = []
        self.pose_publishers = {}

        self.create_subscription(CameraInfo, '/head_front_camera/rgb/camera_info', self.camera_info_callback, 10)
        self.create_subscription(Image, '/head_front_camera/rgb/image_raw', self.image_callback, 10)

        self.get_logger().info("Aruco Pose Estimator Avviato")

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("Camera info ricevuto")

    def image_callback(self, msg):
        if self.camera_matrix is None:
            self.get_logger().warn("Camera info non ricevuto, non posso processare le immagini")
            return
        
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(gray, corners, ids)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)

            for i, marker_id in enumerate(ids.flatten()):
                if marker_id not in self.detected_ids:
                    if len(self.detected_ids) < 4:
                        self.detected_ids.append(marker_id)
                        topic = f'/aruco_marker/ID{marker_id}/pose'
                        self.pose_publishers[marker_id] = self.create_publisher(PoseStamped, topic, 10)
                        self.get_logger().info(f"Publisher creato per il marker {marker_id} su {topic}")
                    else:
                        continue

            if marker_id in self.pose_publishers:
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]
                pose_msg = self.build_pose_stamped(tvec, rvec, msg.header.stamp) # header stamp rappresenta il tempo del messaggio 
                self.pose_publishers[marker_id].publish(pose_msg)

                # cv2.aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03) 
                self.get_logger().info(f"Marker {marker_id} rilevato: posizione {tvec}, orientamento {rvec}")

        # Visualizza l'immagine con i marker e gli assi disegnati
        cv2.imshow("Aruco Detection", gray)
        cv2.waitKey(1) 

    def build_pose_stamped(self, tvec, rvec, stamp_ros2):
        pose = PoseStamped()
        pose.header.frame_id = "head_front_camera_rgb_optical_frame"
        pose.header.stamp = stamp_ros2

        pose.pose.position.x = tvec[0]
        pose.pose.position.y = tvec[1]
        pose.pose.position.z = tvec[2]

        rotation_matrix, _ = cv2.Rodrigues(rvec)
        quat = tf_transformations.quaternion_from_matrix(np.vstack([
            np.hstack([rotation_matrix, np.array([[0], [0], [0]])]),
            np.array([[0, 0, 0, 1]])
        ]))

        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        return pose
    
def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


