import rospy
import cv2
import numpy as np
from cv2 import aruco
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import time

class ArucoTracker:
    def __init__(self):
        self.bridge = CvBridge()
        self.prev_position = None

        # Tamaño del marcador en metros
        self.marker_size = 0.05  # 5 cm

        # Calibración de la cámara (reemplazar por tus propios valores si tienes)
        self.camera_matrix = np.array([[800, 0, 320],
                                       [0, 800, 240],
                                       [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1))  # Suponemos sin distorsión

        # Diccionario y parámetros ArUco
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)  # Cambié esto
        self.parameters = aruco.DetectorParameters_create()

        # Suscriptor a la imagen
        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)

        # Publicador para el seguimiento
        self.pub_aruco = rospy.Publisher("/aruco_data", Float32MultiArray, queue_size=10)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detectar marcadores ArUco
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None and len(corners) > 0:
            aruco.drawDetectedMarkers(frame, corners, ids)

            for i in range(len(ids)):
                c = corners[i][0]
                center_x = int(np.mean(c[:, 0]))
                center_y = int(np.mean(c[:, 1]))

                # Estimar pose 3D
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(c, self.marker_size, self.camera_matrix, self.dist_coeffs)
                rvec, tvec = rvecs[0], tvecs[0]
                distance = np.linalg.norm(tvec)

                # Mostrar ID y posición
                cv2.putText(frame, f"ID: {ids[i][0]}", (center_x + 10, center_y - 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame, f"Pos: ({center_x},{center_y})", (center_x + 10, center_y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                cv2.putText(frame, f"Dist: {distance:.2f} m", (center_x + 10, center_y + 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

                # Mostrar dirección si hay posición previa
                if self.prev_position:
                    dx = center_x - self.prev_position[0]
                    dy = center_y - self.prev_position[1]
                    dir_x = "Right" if dx > 0 else "Left" if dx < 0 else "Still"
                    dir_y = "Down" if dy > 0 else "Up" if dy < 0 else "Still"
                    cv2.putText(frame, f"Dir: {dir_x}, {dir_y}", (center_x + 10, center_y + 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 100, 255), 2)

                self.prev_position = (center_x, center_y)

                # Publicar datos en ROS
                msg = Float32MultiArray()
                msg.data = [ids[i][0], center_x, center_y, distance, *rvec.tolist(), *tvec.tolist()]
                self.pub_aruco.publish(msg)

        cv2.imshow("Aruco Tracker", frame)
        cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node('aruco_tracker_node', anonymous=True)
    tracker = ArucoTracker()
    rospy.spin()
    cv2.destroyAllWindows()

