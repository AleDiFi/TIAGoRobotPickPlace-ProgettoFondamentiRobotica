import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import tf2_ros

class ArucoPoseEstimator(Node):
    """ classe per stimare la posa di marker ArUco e pubblicare le pose trasformate """
    def __init__(self):
        super().__init__('aruco_pose_estimator') 

        self.bridge = CvBridge() # Converte le immagini ROS in OpenCV
        self.marker_size = 0.06  # Marker size in metri

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250) 
        self.aruco_params = cv2.aruco.DetectorParameters() 

        self.camera_matrix = None
        self.dist_coeffs = None

        self.detected_ids = [] # Lista per tenere traccia degli ID dei marker rilevati
        self.pose_publishers = {} #dizionario per i publisher delle pose
        self.transformed_pose_publishers = {}  # Nuovo dizionario per i publisher delle pose trasformate
        self.last_poses = {} # Dizionario per le ultime pose rilevate
        self.last_transformed_poses = {}  # Dizionario per le pose trasformate

        self.max_markers = 4
        for i in range(self.max_markers):
            pose_topic = f'/aruco_marker/ID{i+1}/pose' # topic per la posa del marker
            transformed_pose_topic = f'/aruco_marker/ID{i+1}/transformed_pose' # topic per la posa trasformata

        
        """creazione dei publisher per le pose e le pose trasformate"""

        self.pose_publishers[i+1] = self.create_publisher(PoseStamped, pose_topic, 10) # publisher per la posa
        self.transformed_pose_publishers[i+1] = self.create_publisher(PoseStamped, transformed_pose_topic, 10) # publisher per la posa trasformata

        self.get_logger().info(f"Publisher pre-creato per marker {i+1} su {pose_topic} e {transformed_pose_topic}") #publisher per la posa e la posa trasformata
        self.create_subscription(CameraInfo, '/head_front_camera/rgb/camera_info', self.camera_info_callback, 10) #subscription per le informazioni della camera
        self.create_subscription(Image, '/head_front_camera/rgb/image_raw', self.image_callback, 10) #subscription per le immagini della camera

        # Timer per ripubblicare le ultime pose rilevate ogni 1 secondi
        self.timer = self.create_timer(1.0, self.publish_last_poses)
        self.get_logger().info("Aruco Pose Estimator Avviato")

        # Listener per ascoltare le trasformazioni tra i frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def camera_info_callback(self, msg):

        """ funzione di callback per le informazioni della camera. 
        Riceve le informazioni della camera e le memorizza nella matrice della camera e nei coefficienti di distorsione 
        
        input: msg: CameraInfo"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3) 
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("Camera info ricevuto")

    def image_callback(self, msg):

        """ funzione di callback per le immagini.
        Riceve le immagini dalla camera e rileva i marker ArUco.
        Se i marker sono stati rilevati, calcola la posa e la pubblica 
        
        input: msg: Image"""
        if self.camera_matrix is None:
            self.get_logger().warn("Camera info non ricevuto, non posso processare le immagini")
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') # Converte l'immagine ROS in un'immagine OpenCV
        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params) # Rileva i marker ArUco nell'immagine

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids) # Disegna i marker rilevati nell'immagine

            
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)# Stima la posa dei marker rilevati
            """ rvecs e tvecs sono le rotazioni e le traslazioni stimate per ogni marker """


            for i, marker_id in enumerate(ids.flatten()): 
                if marker_id not in self.detected_ids:
                    """se il marker è già stato rilevato, non lo aggiungere alla lista dei rilevati"""

                    if len(self.detected_ids) < self.max_markers: 
                        self.detected_ids.append(marker_id)
                        self.get_logger().info(f"Marker {marker_id} aggiunto alla lista dei rilevati.")
                    else:
                        self.get_logger().info(
                            f"\n📍 Marker ID: {marker_id}\n"
                            f"   ➤ Posizione (x, y, z): [{tvec[0]:.3f}, {tvec[1]:.3f}, {tvec[2]:.3f}]\n"
                            f"   ➤ Rotazione (rvec):   [{rvec[0]:.3f}, {rvec[1]:.3f}, {rvec[2]:.3f}]\n"
                            "--------------------------------------------------"
                        )
                        continue

                if marker_id in self.pose_publishers: 
                    """se il marker è già stato rilevato e il publisher è stato creato, pubblica la posa"""
                    rvec = rvecs[i][0]
                    tvec = tvecs[i][0]
                    pose_msg, quat = self.build_pose_stamped(tvec, rvec, msg.header.stamp) # Crea il messaggio PoseStamped
                    self.last_poses[marker_id] = pose_msg # memorizza l'ultima posa rilevata
                    self.pose_publishers[marker_id].publish(pose_msg) # pubblica la posa
                    self.get_logger().info(f"Marker {marker_id} rilevato: posizione {tvec}, orientamento {rvec}\n") #  stampa la posa
                    
                    transformed_pose_msg = self.build_transform_pose(pose_msg, msg.header.stamp, quat) # Crea il messaggio PoseStamped trasformato
                    if marker_id in self.transformed_pose_publishers:
                        # se il publisher per la posa trasformata è stato creato, pubblica la posa trasformata
                        self.transformed_pose_publishers[marker_id].publish(transformed_pose_msg) # pubblica la posa trasformata
                        self.last_transformed_poses[marker_id] = transformed_pose_msg # memorizza l'ultima posa trasformata
                        tp = transformed_pose_msg.pose.position # posizione trasformata
                        to = transformed_pose_msg.pose.orientation # orientamento trasformato
                        self.get_logger().info(
                            f"\n🔄 Trasformazione Marker ID: {marker_id}\n"
                            f"   ➤ Posizione trasformata (x, y, z): [{tp.x:.3f}, {tp.y:.3f}, {tp.z:.3f}]\n"
                            f"   ➤ Orientamento (quaternione):      [x: {to.x:.3f}, y: {to.y:.3f}, z: {to.z:.3f}, w: {to.w:.3f}]\n"
                            "---Roger-Roger------------------------------------"
                        )

                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], self.marker_size / 2) 
                # disegna un sistema di assi 3D (X, Y, Z) sopra il marker, basato sulla posa stimata

        # Mostra l'immagine con i marker rilevati
        cv2.imshow("Aruco Detection", frame)
        cv2.waitKey(1)

    def publish_last_poses(self):
        """ funzione per pubblicare le ultime pose rilevate ogni secondo 
        input: self: ArucoPoseEstimator"""
        for marker_id, pose in self.last_poses.items():
            """se il marker è già stato rilevato e il publisher è stato creato, pubblica la posa"""
            if marker_id in self.pose_publishers:
                self.pose_publishers[marker_id].publish(pose)
        for marker_id, transformed_pose in self.last_transformed_poses.items():
            if marker_id in self.pose_publishers:
                self.transformed_pose_publishers[marker_id].publish(transformed_pose)

    def build_pose_stamped(self, tvec, rvec, stamp_ros2):
        """ funzione per costruire il messaggio PoseStamped 
        input: tvec: traslazione del marker
               rvec: rotazione del marker
               stamp_ros2: timestamp ROS2
        output: pose: messaggio PoseStamped"""
        pose.header.frame_id = 'head_front_camera_color_optical_frame'
        pose.header.stamp = stamp_ros2

        # Imposta la posizione iniziale del marker
        pose.pose.position.x = tvec[0]
        pose.pose.position.y = tvec[1]
        pose.pose.position.z = tvec[2]

        # Calcola la rotazione del marker
        rotation_matrix, _ = cv2.Rodrigues(rvec) 
        rot = R.from_matrix(rotation_matrix) 
        quat = rot.as_quat()

        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        return pose, quat

    def build_transform_pose(self, pose, stamp_ros2, quat):
        
        """ funzione per costruire il messaggio PoseStamped trasformato 
        input: pose: messaggio PoseStamped
               stamp_ros2: timestamp ROS2
               quat: quaternione del marker
        output: transformed_pose: messaggio PoseStamped trasformato"""
        try:
            transformed_pose = PoseStamped()
            transformed_pose.header.frame_id = "base_footprint"
            transformed_pose.header.stamp = stamp_ros2


            # Effettua il lookup della trasformazione tra base_footprint e head_front_camera_link
            transform = self.tf_buffer.lookup_transform('base_footprint', 'head_front_camera_color_optical_frame', rclpy.time.Time())

            # Estrai la traslazione e la rotazione della trasformazione
            translation = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])
            quat_transform = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]
            rotation_head = R.from_quat(quat_transform).as_matrix() # Matrice di rotazione della testa rispetto alla base

            # Trasforma la posizione del marker
            marker_position = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
            transformed_position = rotation_head @ marker_position + translation

            # Aggiorna la posizione trasformata
            transformed_pose.pose.position.x = transformed_position[0]
            transformed_pose.pose.position.y = transformed_position[1]
            transformed_pose.pose.position.z = transformed_position[2]

            # Componi le rotazioni (rotazione del marker rispetto alla base)
            marker_rotation = R.from_quat(quat)
            transformed_rotation = R.from_matrix(rotation_head) * marker_rotation
            transformed_quat = transformed_rotation.as_quat()

            # Aggiorna l'orientamento trasformato
            transformed_pose.pose.orientation.x = transformed_quat[0]
            transformed_pose.pose.orientation.y = transformed_quat[1]
            transformed_pose.pose.orientation.z = transformed_quat[2]
            transformed_pose.pose.orientation.w = transformed_quat[3]

            # Pubblica la posizione e l'orientamento trasformati sul topic dedicato
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Errore nella trasformazione: {e}")

        return transformed_pose

        


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()