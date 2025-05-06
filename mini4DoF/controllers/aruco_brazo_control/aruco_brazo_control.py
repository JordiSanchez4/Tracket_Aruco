#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Joy
import numpy as np

class ArucoFollower:
    def __init__(self):
        rospy.init_node('aruco_brazo_control', anonymous=True)

        rospy.Subscriber('/aruco_position', Point, self.aruco_callback)
        self.control_pub = rospy.Publisher('/control', Joy, queue_size=10)

        self.k = 1.0  # Ganancia del controlador
        self.dt = 1.0 / 30  # Tiempo de muestreo (asumiendo 30 Hz)
        self.current_pos = np.array([2.5, 2.8, 0.76])

    def aruco_callback(self, msg):
        target = np.array([msg.x, msg.y, msg.z])
        error = target - self.current_pos
        v_ee = self.k * error

        rospy.loginfo(f"[ARUCO] Posición recibida: x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}")
        rospy.loginfo(f"[CONTROL] v_ee enviado al brazo: {v_ee}")

        joy_msg = Joy()
        joy_msg.header.stamp = rospy.Time.now()
        joy_msg.header.frame_id = "base_link"
        joy_msg.axes = [v_ee[0], v_ee[1], v_ee[2], 0.0]
        self.control_pub.publish(joy_msg)

        # Estimación local de la posición (sólo para simular)
        self.current_pos += v_ee * self.dt

if __name__ == '__main__':
    try:
        follower = ArucoFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

