from controller import Supervisor
import random
import rospy
from geometry_msgs.msg import Point
import math

class MoverAruco:
    def __init__(self):
        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.offset = [2.39219, 2.95957, 0.74]

        # Inicialización del nodo ROS
        rospy.init_node('aruco_mover', disable_signals=True)
        self.pub = rospy.Publisher('/aruco_position', Point, queue_size=10)

        self.aruco_node = self.robot.getFromDef("ARUCO_MARKER")
        if self.aruco_node is None:
            print("[ERROR] No se encontró el ArUco en el mundo.")
            exit()

    def move(self):
        contador_tiempo = 0
        esperar_steps = int((20000) / self.timestep)  # Cambia cada 2 segundos

        # Ángulo de rotación en radianes
        theta = math.radians(45)
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)

        while self.robot.step(self.timestep) != -1 and not rospy.is_shutdown():
            if contador_tiempo == 0:
                # Movimiento aleatorio dentro del rango
                x = random.uniform(self.offset[0] + 0.04, self.offset[0] + 0.15)
                y = random.uniform(self.offset[1] + 0.04, self.offset[1] + 0.15)

                nueva_pos = [x, y, 0.76]
                self.aruco_node.getField("translation").setSFVec3f(nueva_pos)

                # Cálculo del desplazamiento desde el offset
                dx = x - self.offset[0]
                dy = y - self.offset[1]
                dz = 0

                # Rotar las coordenadas alrededor de Z 45 grados
                dx_rot = dx * cos_theta - dy * sin_theta
                dy_rot = dx * sin_theta + dy * cos_theta

                print(f"[INFO] ArUco coordenadas rotadas publicadas: {[dx_rot, dy_rot, dz]}")

                # Publicar la posición rotada en ROS
                point_msg = Point(x=dx_rot, y=dy_rot, z=dz)
                self.pub.publish(point_msg)

            contador_tiempo += 1
            if contador_tiempo >= esperar_steps:
                contador_tiempo = 0

if __name__ == "__main__":
    mover = MoverAruco()
    mover.move()
