
#!/usr/bin/env python3
import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
import tf
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R

class ArucoPosePublisher:
    def __init__(self):
        rospy.init_node("aruco_pose_publisher")
        self.camera_matrix = np.array([
            [558.16434798, 0.0, 306.13849723],
            [0.0, 520.24701933, 217.56454986],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)
        self.dist_coeffs = np.array([-0.0252525417, -0.461275080, 0.0400812861, -0.000199458565, 0.777505442])
        self.marker_length = 0.01669
        self.bridge = CvBridge()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()
        self.pub_point = rospy.Publisher("/aruco_position", PointStamped, queue_size=10)
        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        rospy.loginfo("Aruco Pose Publisher Node Running")
        rospy.spin()

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        self.tf_broadcaster.sendTransform(
            (0.0, 0.0, 0.0),
            tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time.now(),
            "camera_link",
            "world"
        )

        if ids is not None:
            for i in range(len(ids)):
                image_points = corners[i][0]
                cv2.putText(frame, f"ID: {ids[i][0]}", (10, 60 + 30 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                object_points = np.array([
                    [-self.marker_length/2,  self.marker_length/2, 0],
                    [ self.marker_length/2,  self.marker_length/2, 0],
                    [ self.marker_length/2, -self.marker_length/2, 0],
                    [-self.marker_length/2, -self.marker_length/2, 0]
                ], dtype=np.float32)

                retval, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.dist_coeffs)
                if not retval:
                    continue

                rot_matrix, _ = cv2.Rodrigues(rvec)
                rot = R.from_matrix(rot_matrix)
                quat = rot.as_quat()

                self.tf_broadcaster.sendTransform(
                    (tvec[0][0], tvec[1][0], tvec[2][0]),
                    (quat[0], quat[1], quat[2], quat[3]),
                    rospy.Time.now(),
                    "aruco_marker",
                    "camera_link"
                )

                point_msg = PointStamped()
                point_msg.header.stamp = rospy.Time.now()
                point_msg.header.frame_id = "camera_link"
                point_msg.point.x = tvec[0][0]
                point_msg.point.y = tvec[1][0]
                point_msg.point.z = tvec[2][0]
                self.pub_point.publish(point_msg)

                aruco.drawDetectedMarkers(frame, corners)
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)

        cv2.imshow("Aruco View", frame)
        cv2.waitKey(1)

if __name__ == "__main__":
    try:
        ArucoPosePublisher()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()
