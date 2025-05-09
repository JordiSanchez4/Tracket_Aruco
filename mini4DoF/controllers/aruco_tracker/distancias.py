import rclpy
import cv2
import time
import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos, pi
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from transforms3d.euler import euler2quat
import math

class TFAndPointPublisher(Node):
    def _init_(self):
        super()._init_('tf_and_points_publisher')

        # Broadcaster de TF
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer para publicar periódicamente TF
        self.timer = self.create_timer(0.1, self.publish_tf)

        # Variables de animación (si las necesitas)
        self.angle = 0.0

    def publish_tf(self):
        # Publicar TF para camera_link
        camera_transform = TransformStamped()
        camera_transform.header.stamp = self.get_clock().now().to_msg()
        camera_transform.header.frame_id = 'world'
        camera_transform.child_frame_id = 'camera'
        camera_transform.transform.translation.x = 0.360 #m
        camera_transform.transform.translation.y = 0.00 #m 
        camera_transform.transform.translation.z = 0.410 #m 

        roll = -math.pi/2
        pitch = 0
        yaw = 0

         # Convertir Euler a cuaterniones
        #quat = euler2quat(roll, pitch, yaw, axes='sxyz')
        quat = [0.0, 0.0, -1.0, 0.0]
        camera_transform.transform.rotation.x = quat[1]
        camera_transform.transform.rotation.y = quat[2]
        camera_transform.transform.rotation.z = quat[3]
        camera_transform.transform.rotation.w = quat[0]
         
        

        self.tf_broadcaster.sendTransform(camera_transform)

def main(args=None):

    rclpy.init(args=args)
    node = TFAndPointPublisher()


    try:
        # Mantener ROS 2 ejecutándose en el hilo principal
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if _name_ == '_main_':
    main()
