#!/usr/bin/env python3
from controller import Robot
import rospy
from geometry_msgs.msg import Point

class WineglassController:
    def __init__(self):
        # Inicializa el robot y su tiempo de simulación
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Accede al nodo de ROS
        rospy.init_node('wineglass_controller_node', anonymous=True)
        
        # Publicador de coordenadas
        self.pub = rospy.Publisher("/wineglass_coordinates", Point, queue_size=10)

        # Accede al objeto 'Wineglass'
        self.wineglass = self.robot.getFromDef("Wineglass")

    def run(self):
        while self.robot.step(self.timestep) != -1:
            # Obtiene la posición del Wineglass
            translation_field = self.wineglass.getField("translation")
            position = translation_field.getSFVec3f()

            # Publica las coordenadas en ROS
            point_msg = Point()
            point_msg.x = position[0]
            point_msg.y = position[1]
            point_msg.z = position[2]
            self.pub.publish(point_msg)

            # Aquí puedes mover el Wineglass (por ejemplo, hacer que se mueva hacia nuevas coordenadas)
            new_position = [position[0] + 0.01, position[1], position[2]]  # Mover ligeramente en el eje X
            translation_field.setSFVec3f(new_position)

if __name__ == "__main__":
    controller = WineglassController()
    controller.run()

