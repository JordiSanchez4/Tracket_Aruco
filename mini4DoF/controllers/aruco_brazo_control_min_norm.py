#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Joy
import numpy as np

class ArucoFollower:
    def __init__(self):
        rospy.init_node('aruco_brazo_control', anonymous=True)

        # Suscripción a la posición del ArUco
        rospy.Subscriber('/aruco_position', Point, self.aruco_callback)

        # Suscripción a los estados actuales del brazo
        rospy.Subscriber('/states', Joy, self.state_callback)

        # Publicación de velocidades hacia el brazo
        self.control_pub = rospy.Publisher('/control', Joy, queue_size=10)

        # Ganancia del controlador proporcional
        self.k = 1.0

        # Velocidades máximas permitidas para los motores
        self.max_velocity = 5.96903  # Ajustar esto a la velocidad máxima de los motores en Webots

        # Posición inicial del efector final (puede venir del estado real del brazo)
        self.current_pos = np.array([2.5, 2.8, 0.76])

    def state_callback(self, msg):
        # ACTUALIZACIÓN: asumimos que los tres primeros elementos son q1, q2, q3
        # Solo funciona si tus articulaciones están alineadas con X, Y, Z respectivamente
        self.current_pos = np.array([msg.axes[0], msg.axes[1], msg.axes[2]])

    def aruco_callback(self, msg):
        # Coordenadas del marcador ArUco
        target = np.array([msg.x, msg.y, msg.z])

        # Calculamos error entre la posición deseada (Aruco) y la actual del efector
        error = target - self.current_pos

        # Control proporcional en el espacio cartesiano
        v_ee = self.k * error

        # Normalizamos las velocidades para que no superen la velocidad máxima de los motores
        norm_v_ee = np.linalg.norm(v_ee)
        if norm_v_ee > self.max_velocity:
            v_ee = (v_ee / norm_v_ee) * self.max_velocity

        # DEBUG: Imprimir velocidades calculadas
        rospy.loginfo(f"[CONTROL] v_ee enviado al brazo: {v_ee}")

        # Publicar velocidades como si fueran velocidades articulares
        joy_msg = Joy()
        joy_msg.header.stamp = rospy.Time.now()
        joy_msg.header.frame_id = "base_link"
        joy_msg.axes = [v_ee[0], v_ee[1], v_ee[2], 0.0]  # Velocidades para 4 articulaciones
        self.control_pub.publish(joy_msg)

        # Actualizamos posición actual (simulada)
        self.current_pos += v_ee * 0.033  # sample time = 1/30

if __name__ == '__main__':
    try:
        follower = ArucoFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

