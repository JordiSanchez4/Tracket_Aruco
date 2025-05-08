import rospy
from std_msgs.msg import Float32MultiArray
import cv2
import cv2.aruco as aruco
import numpy as np
from controller import Robot, Camera

class CameraArucoPublisher:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.camera = self.robot.getDevice('WebCam')
        self.camera.enable(self.timestep)

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()  # ← corrección aquí
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)

        rospy.init_node('aruco_camera_publisher', anonymous=True)
        self.pub = rospy.Publisher('/aruco_position_camara', Float32MultiArray, queue_size=10)

    def run(self):
        while self.robot.step(self.timestep) != -1:
            img = self.camera.getImage()
            img_array = np.frombuffer(img, dtype=np.uint8).reshape((self.camera.getHeight(), self.camera.getWidth(), 4))
            frame = cv2.cvtColor(img_array, cv2.COLOR_BGRA2BGR)

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.detector.detectMarkers(gray)

            if ids is not None and len(corners) > 0:
                c = corners[0][0]
                center_x = int((c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4)
                center_y = int((c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4)

                msg = Float32MultiArray()
                msg.data = [center_x, center_y]
                self.pub.publish(msg)

                print(f"[✔] ArUco detectado en: {center_x}, {center_y}")

if __name__ == "__main__":
    viewer = CameraArucoPublisher()
    viewer.run()
