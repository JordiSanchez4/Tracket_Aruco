#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import tf
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import yaml
from scipy.spatial.transform import Rotation as R

ARUCO_DICT = cv2.aruco.DICT_4X4_50  # Usa el que estés utilizando

class ArucoTfPublisher:
    def __init__(self):
        rospy.init_node('aruco_tf_publisher_node')

        # Parámetros
        self.marker_length = rospy.get_param("~marker_length", 0.05)  # en metros
        calibration_file = rospy.get_param("~calibration_file", "/home/usuario/calibration.yaml")
        self.marker_frame = rospy.get_param("~marker_frame", "aruco_marker")
        self.camera_frame = rospy.get_param("~camera_frame", "camera_frame")
        image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")

        # Leer calibración
        with open(calibration_file, 'r') as f:
            calib_data = yaml.safe_load(f)
            self.camera_matrix = np.array(calib_data['K']).reshape((3, 3))
            self.dist_coeffs = np.array(calib_data['D'])

        # ArUco
        self.aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # ROS
        self.bridge = CvBridge()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.sub = rospy.Subscriber(image_topic, Image, self.image_callback)

        rospy.loginfo("Aruco TF Broadcaster Node Started.")
        rospy.spin()

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs)

            for i in range(len(ids)):
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]

                rot_matrix, _ = cv2.Rodrigues(rvec)
                rot = R.from_matrix(rot_matrix)
                quat = rot.as_quat()  # x, y, z, w

                # Publicar TF
                self.tf_broadcaster.sendTransform(
                    (tvec[0], tvec[1], tvec[2]),
                    (quat[0], quat[1], quat[2], quat[3]),
                    rospy.Time.now(),
                    self.marker_frame,
                    self.camera_frame
                )

                # Visual
                cv2.aruco.drawDetectedMarkers(frame, corners)
                cv2.aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)

        cv2.imshow("Aruco Tracker", frame)
        cv2.waitKey(1)

if __name__ == "__main__":
    try:
        ArucoTfPublisher()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()

