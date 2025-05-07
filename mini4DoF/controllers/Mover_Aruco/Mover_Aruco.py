from controller import Supervisor
import random
import rospy
from geometry_msgs.msg import Point


class MoverAruco:
    def __init__(self):
        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.offset = [2.39219, 2.95957, 0.74]

        # ROS Node
        rospy.init_node('aruco_mover', disable_signals=True)
        self.pub = rospy.Publisher('/aruco_position', Point, queue_size=10)
        
        self.aruco_node = self.robot.getFromDef("ARUCO_MARKER")
        if self.aruco_node is None:
            print("[ERROR] No se encontró el ArUco en el mundo.")
            exit()

    def move(self):
        contador_tiempo = 0
        esperar_steps = int((2000) / self.timestep)  # Cambia cada 2 segundos

        while self.robot.step(self.timestep) != -1 and not rospy.is_shutdown():
            if contador_tiempo == 0:
                x = random.uniform(2.6, 2.66)
                y = random.uniform(2.74, 2.79)
                nueva_pos = [x, y, 0.76]
                self.aruco_node.getField("translation").setSFVec3f(nueva_pos)
                print(f"[INFO] ArUco movido a: {nueva_pos}")
                dx = x-self.offset[0]
                dy = y-self.offset[0]
                dz = 0

                # Publicar posición
                point_msg = Point(x=dx, y=dy, z=dz)
                self.pub.publish(point_msg)

            contador_tiempo += 1
            if contador_tiempo >= esperar_steps:
                contador_tiempo = 0

if __name__ == "__main__":
    mover = MoverAruco()
    mover.move()
