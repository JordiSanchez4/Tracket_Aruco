#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
import numpy as np

def send_manual_velocity(u_manual):
    pub = rospy.Publisher("/control", Joy, queue_size=10)
    rospy.init_node("manual_velocity_sender", anonymous=True)
    
    rate = rospy.Rate(30)  # Frecuencia de envío (30 Hz)

    control_msg = Joy()
    control_msg.header.frame_id = "base_link"

    while not rospy.is_shutdown():
        control_msg.header.stamp = rospy.Time.now()
        control_msg.axes = u_manual  # Tu vector de velocidades
        pub.publish(control_msg)
        rate.sleep()

if __name__ == "__main__":
    try:
        # Define tus velocidades articulares aquí (q1_dot, q2_dot, q3_dot, q4_dot)
        u_manual = [0.5, 0.1, 0.5, 0.5]  # Cambia estos valores según lo que quieras mover
        send_manual_velocity(u_manual)
    except rospy.ROSInterruptException:
        pass
