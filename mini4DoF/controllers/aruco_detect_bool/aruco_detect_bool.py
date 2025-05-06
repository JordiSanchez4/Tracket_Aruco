import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Point

class ArucoDetectionChecker:
    def __init__(self):
        rospy.init_node('aruco_detection_checker', anonymous=True)

        # Publicador de detección
        self.pub = rospy.Publisher('/aruco_detected', Bool, queue_size=10)

        # Suscriptor de posición ArUco
        rospy.Subscriber('/aruco_position', Point, self.aruco_callback)

        self.detected = False

        rospy.spin()

    def aruco_callback(self, msg):
        # Si recibimos alguna posición, consideramos que el ArUco está detectado
        if msg.x != 0.0 or msg.y != 0.0 or msg.z != 0.0:
            self.detected = True
        else:
            self.detected = False

        # Publicamos el estado
        detection_msg = Bool()
        detection_msg.data = self.detected
        self.pub.publish(detection_msg)

        # Log
        rospy.loginfo(f"[INFO] ArUco detectado: {self.detected}")

if __name__ == '__main__':
    try:
        ArucoDetectionChecker()
    except rospy.ROSInterruptException:
        pass

