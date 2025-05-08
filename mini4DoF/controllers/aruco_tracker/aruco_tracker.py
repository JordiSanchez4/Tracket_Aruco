import rospy
import cv2
import numpy as np
from cv2 import aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

class ArucoTracker:
    def __init__(self):
        self.bridge = CvBridge()
        self.prev_position = None
        self.prev_time = time.time()

        self.marker_size = 0.05  # 5 cm
        self.focal_length = 800

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)

        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None and len(corners) > 0:
            for i in range(len(ids)):
                c = corners[i][0]
                aruco.drawDetectedMarkers(frame, [corners[i]], ids[i])

                center_x = int(np.mean(c[:, 0]))
                center_y = int(np.mean(c[:, 1]))

                marker_width_pixels = np.linalg.norm(c[0] - c[1])
                distance = (self.focal_length * self.marker_size) / marker_width_pixels

                dx, dy = 0, 0
                dir_x, dir_y = "Stationary", "Stationary"
                if self.prev_position is not None:
                    dx = center_x - self.prev_position[0]
                    dy = center_y - self.prev_position[1]
                    dir_x = "Right" if dx > 0 else "Left" if dx < 0 else "Stationary"
                    dir_y = "Down" if dy > 0 else "Up" if dy < 0 else "Stationary"
                self.prev_position = (center_x, center_y)

                # ------ Dibujar fondo semitransparente ------
                box_x, box_y = center_x + 20, center_y - 40
                text_lines = [
                    f"ID: {ids[i][0]}",
                    f"Pos: ({center_x}, {center_y})",
                    f"Dist: {distance:.2f} m",
                    f"Move: X-{dir_x}, Y-{dir_y}"
                ]
                padding = 5
                line_height = 20
                box_width = 200
                box_height = line_height * len(text_lines) + padding * 2

                overlay = frame.copy()
                cv2.rectangle(overlay, (box_x, box_y),
                              (box_x + box_width, box_y + box_height),
                              (50, 50, 50), -1)
                alpha = 0.5
                frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)

                # ------ Dibujar texto ------
                for j, text in enumerate(text_lines):
                    text_pos = (box_x + padding, box_y + (j + 1) * line_height)
                    cv2.putText(frame, text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.imshow("Aruco Tracker", frame)
        cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node('aruco_tracker_node', anonymous=True)
    tracker = ArucoTracker()
    rospy.spin()
    cv2.destroyAllWindows()

