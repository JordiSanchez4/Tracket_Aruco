#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Joy
import numpy as np
from scipy.io import savemat
import os
import threading



# Global variables Odometry Drone
posicion_aruco = np.array([0.15,0.15,0.04])
aruco_lock = threading.Lock()
q1 = 1.0
q2 = 0
q3 = 0.0
q4 = 0.0
q1_p = 0
q2_p = 0
q3_p = 0.0
q4_p = 0.0

#Caracteristicas del Brazo
l = [0.0676, 0.06883, 0.06883, 0.15916]


def CDArm4DOF(l, q):
    q1, q2, q3, q4 = q

    hx = np.cos(q1)*(l[2]*np.cos(q2 + q3) + l[1]*np.cos(q2) + l[3]*np.cos(q2 + q3 + q4))
    hy = np.sin(q1)*(l[2]*np.cos(q2 + q3) + l[1]*np.cos(q2) + l[3]*np.cos(q2 + q3 + q4))
    hz = l[0] + l[2]*np.sin(q2 + q3) + l[1]*np.sin(q2) + l[3]*np.sin(q2 + q3 + q4) - 0.05  # Ajusta -0.05 a lo que necesites

    cinematica4DOF = np.array([hx, hy, hz])
    return cinematica4DOF

def jacobiana_Brazo4DOF(L, q):
    l1, l2, l3, l4 = L
    q1, q2, q3, q4 = q

    J = np.array([
        [-np.sin(q1)*(l3*np.cos(q2 + q3) + l2*np.cos(q2) + l4*np.cos(q2 + q3 + q4)), 
         -np.cos(q1)*(l3*np.sin(q2 + q3) + l2*np.sin(q2) + l4*np.sin(q2 + q3 + q4)), 
         -np.cos(q1)*(l3*np.sin(q2 + q3) + l4*np.sin(q2 + q3 + q4)), 
         -l4*np.sin(q2 + q3 + q4)*np.cos(q1)],
        
        [np.cos(q1)*(l3*np.cos(q2 + q3) + l2*np.cos(q2) + l4*np.cos(q2 + q3 + q4)), 
         -np.sin(q1)*(l3*np.sin(q2 + q3) + l2*np.sin(q2) + l4*np.sin(q2 + q3 + q4)), 
         -np.sin(q1)*(l3*np.sin(q2 + q3) + l4*np.sin(q2 + q3 + q4)), 
         -l4*np.sin(q2 + q3 + q4)*np.sin(q1)],
        
        [0, 
         l3*np.cos(q2 + q3) + l2*np.cos(q2) + l4*np.cos(q2 + q3 + q4), 
         l3*np.cos(q2 + q3) + l4*np.cos(q2 + q3 + q4), 
         l4*np.cos(q2 + q3 + q4)]
    ])
    
    return J


def Controler_pos(L, q, he, hdp,val):

    q1, q2, q3, q4 = q
    print(f"q1:{q1},q2:{q2},q3:{q3},q4:{q4}")

    # Jacobiano
    J = jacobiana_Brazo4DOF(L, q)

    # Matriz de ganancia
    K = val*np.eye(3)

    # Posiciones deseadas de los eslabones
    q1d = 0 * np.pi / 180
    q2d = 30 * np.pi / 180
    q3d = -15 * np.pi / 180
    q4d = +30 * np.pi / 180

    # Vector n
    hq1 = q1d - q1
    hq2 = q2d - q2
    hq3 = q3d - q3
    hq4 = q4d - q4
    n = np.array([hq1, hq2, hq3, hq4])

    #print(f"Posiciones deseadas: Xd = {q1d}, Yd = {q2d}, Zd = {q3d}, Wd = {q4d}")

    # Matriz ganancia D
    D = np.diag([1,5,5,10])

    # Tarea secundaria para el robot
    I = np.eye(4)
    
    #TAREA_S = np.dot((I - np.dot(np.linalg.pinv(J), J)), np.dot(D, n))

    # Matriz de ganancia
    K = val*np.eye(3)


    # Control principal con regularización
    try:
        Vref = np.linalg.lstsq(J, K @ np.tanh(0.5 * he), rcond=1e-3)[0] #+ TAREA_S
    except:
        Vref = np.zeros(4)
    # Controlador
    max_vel = 5.9
    Vref = np.clip(Vref, -max_vel, max_vel)
    
    return Vref


def states_call_back(state_msg):
    print(f"state_msg:{state_msg}")
    global q1, q2, q3, q4, q1_p, q2_p, q3_p, q4_p
    # Leer velocidades lineales deseadas del mensaje
    q1 = state_msg.axes[0]
    q2 = state_msg.axes[1]
    q3 = state_msg.axes[2]
    q4 = state_msg.axes[3]
    q1_p = state_msg.axes[4]
    q2_p = state_msg.axes[5]
    q3_p = state_msg.axes[6]
    q4_p = state_msg.axes[7]

def get_pose_arm():
    global q1, q2, q3, q4
    x = [q1, q2, q3, q4]
    return x

def get_vel_arm():
    global q1_p, q2_p, q3_p, q4_p
    x_p = [q1_p, q2_p, q3_p, q4_p]
    return x_p


def send_velocity_control(u):


    # PUBLISHER
    control_msg = Joy()
    control_pub = rospy.Publisher("/control", Joy, queue_size=10)

    control_msg.header.frame_id = "base_link"
    control_msg.header.stamp = rospy.Time.now()

    control_msg.axes = u

    # Publish control values
    control_pub.publish(control_msg)

def main():
    global posicion_aruco
    # Simulation Time
    t_final = 6000
    # Sample time
    frec= 30
    t_s = 1/frec
       
    # Time simulation
    t = np.arange(0, t_final, t_s)

    # Vector Initial conditions
    x = np.zeros((4, t.shape[0]), dtype = np.double)
    x_p = np.zeros((4, t.shape[0]), dtype = np.double)
    h = np.zeros((3, t.shape[0]+1), dtype = np.double)
    u = np.zeros((4, t.shape[0]+1), dtype = np.double)

    # Read Real data
    x[:, 0] = get_pose_arm()
    x_p[:, 0] = get_vel_arm()

    h[:,0] = CDArm4DOF(l, x[:, 0])

    # Simulation System
    ros_rate = frec  # Tasa de ROS en Hz
    rate = rospy.Rate(ros_rate)  # Crear un objeto de la clase rospy.Rate

    # Errors of the system
    Error = np.zeros((3, t.shape[0]), dtype = np.double)

    # Ganancia del controlador
    K = 0.5

    a = True
    umbral = 0.01
    #INICIALIZA LECTURA DE ODOMETRIA redefim=nir con un while, y hacer una k sumando, o hacer un tiempo muyn grande
    for k in range(0,t.shape[0]):
        with aruco_lock:
            ref = posicion_aruco.copy()
        print(f"Ref:{ref}")
        # Read Real data
        x[:, k] = get_pose_arm()
        x_p[:, k] = get_vel_arm()

        h[:,k] = CDArm4DOF(l, x[:, k])
        print(f"Posicion_actual:{h[:, k]}")
        #Controlador
        Error[:,k] = ref - h[:, k]
        print(f"Error:{Error[:,k]}")
        u[:,k] = Controler_pos(l, x[:, k], Error[:,k], np.zeros(3), K)
        send_velocity_control(u[:,k])
        # Loop_rate.sleep()
        rate.sleep() 
        print(Error[:,k])
    
    send_velocity_control([0, 0, 0, 0])

def ARUCO_POSITION_call_back(msg):
    global posicion_aruco
    with aruco_lock:
        posicion_aruco = np.array([msg.x, msg.y, msg.z])
        print(f"posicion_aruco:{posicion_aruco}")

if __name__ == '__main__':
    try:
         # Node Initialization
        rospy.init_node("Controlador",disable_signals=True, anonymous=True)
        # SUCRIBER
        rospy.Subscriber("/states", Joy, states_call_back)
        rospy.Subscriber("/aruco_position", Point, ARUCO_POSITION_call_back)
       
        TH = threading.Thread(target=main)
        TH.start()
        rospy.spin()
    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("\nError System")
        send_velocity_control([0, 0, 0, 0])
        pass
    else:
        print("Complete Execution")


    