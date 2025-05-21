import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

class ArucoDetector:
    def __init__(self):
        rospy.init_node('aruco_detector', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/webcam/image_raw", Image, self.image_callback)
        self.pos_pub = rospy.Publisher("/aruco_position", Point, queue_size=10)

        # Parámetros de la cámara
        self.camera_matrix = np.array([[1000, 0, 640], [0, 1000, 360], [0, 0, 1]])  # Ajustar según calibración real
        self.dist_coeffs = np.zeros((5, 1))  # Suponemos sin distorsión

        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, self.camera_matrix, self.dist_coeffs)
            for tvec in tvecs:
                x, y, z = tvec[0]
                print(f"[ArucoDetector] Posición detectada: x={x:.3f}, y={y:.3f}, z={z:.3f}")
                point = Point(x=x, y=y, z=z)
                self.pos_pub.publish(point)
                break

if __name__ == "__main__":
    try:
        ArucoDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
