#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point
import numpy as np
from scipy.io import savemat
import threading
import matplotlib.pyplot as plt

# Características del Brazo
l = [0.0676, 0.06883, 0.06883, 0.15916]

# Variables globales
q1 = q2 = q3 = q4 = 0.0
q1_p = q2_p = q3_p = q4_p = 0.0
posicion_aruco = np.array([0.15, 0.15, 0.04])
aruco_lock = threading.Lock()
error_integral = np.zeros(3)
error_anterior = np.zeros(3)

# === Kalman Filter ===
class KalmanFilter:
    def __init__(self, dt=0.02):
        self.x = np.zeros((6, 1))
        self.P = np.eye(6) * 0.01
        self.F = np.block([[np.eye(3), dt*np.eye(3)],
                           [np.zeros((3, 3)), np.eye(3)]])
        self.H = np.hstack([np.eye(3), np.zeros((3, 3))])
        self.R = np.eye(3) * 0.002
        self.Q = np.eye(6) * 0.0001

    def update(self, z):
        z = np.array(z).reshape((3, 1))
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P

        return self.x[:3].flatten()

def CDArm4DOF(l, q):
    q1, q2, q3, q4 = q
    hx = np.cos(q1)*(l[2]*np.cos(q2 + q3) + l[1]*np.cos(q2) + l[3]*np.cos(q2 + q3 + q4))
    hy = np.sin(q1)*(l[2]*np.cos(q2 + q3) + l[1]*np.cos(q2) + l[3]*np.cos(q2 + q3 + q4))
    hz = l[0] + l[2]*np.sin(q2 + q3) + l[1]*np.sin(q2) + l[3]*np.sin(q2 + q3 + q4)
    return np.array([hx, hy, hz])

def jacobiana_Brazo4DOF(L, q):
    l1, l2, l3, l4 = L
    q1, q2, q3, q4 = q
    return np.array([
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

def Controler_pos_PID(L, q, he):
    global error_integral, error_anterior
    Kp = np.array([2.0, 2.0, 2.0])
    Ki = np.array([0.1, 0.1, 0.1])
    Kd = np.array([0.01, 0.01, 0.01])
    dt = 0.02

    error_integral += he * dt
    error_derivativo = (he - error_anterior) / dt
    error_anterior = he

    u_cart = Kp * he + Ki * error_integral + Kd * error_derivativo

    J = jacobiana_Brazo4DOF(L, q)
    Vref = np.linalg.pinv(J) @ u_cart
    return Vref

def states_call_back(state_msg):
    global q1, q2, q3, q4, q1_p, q2_p, q3_p, q4_p
    q1, q2, q3, q4 = state_msg.axes[0:4]
    q1_p, q2_p, q3_p, q4_p = state_msg.axes[4:8]

def ARUCO_POSITION_call_back(msg):
    global posicion_aruco
    with aruco_lock:
        posicion_aruco = np.array([msg.x, msg.y, 0.05])

def get_pose_arm():
    return [q1, q2, q3, q4]

def get_vel_arm():
    return [q1_p, q2_p, q3_p, q4_p]

def send_velocity_control(u):
    control_msg = Joy()
    control_msg.header.frame_id = "base_link"
    control_msg.header.stamp = rospy.Time.now()
    control_msg.axes = u
    control_pub.publish(control_msg)

def main():
    global control_pub
    control_pub = rospy.Publisher("/control", Joy, queue_size=10)
    t_final = 200
    frec = 50
    t_s = 1 / frec
    t = np.arange(0, t_final, t_s)

    x = np.zeros((4, t.shape[0]))
    x_p = np.zeros((4, t.shape[0]))
    h = np.zeros((3, t.shape[0] + 1))
    u = np.zeros((4, t.shape[0] + 1))
    ref = np.zeros((3, t.shape[0]))
    Error = np.zeros((3, t.shape[0]))

    rate = rospy.Rate(frec)
    x[:, 0] = get_pose_arm()
    x_p[:, 0] = get_vel_arm()
    h[:, 0] = CDArm4DOF(l, x[:, 0])

    kalman = KalmanFilter(dt=t_s)

    for k in range(t.shape[0]):
        with aruco_lock:
            ref[:, k] = kalman.update(posicion_aruco.copy())

        x[:, k] = get_pose_arm()
        x_p[:, k] = get_vel_arm()
        h[:, k] = CDArm4DOF(l, x[:, k])
        Error[:, k] = ref[:, k] - h[:, k]

        print(f"[{k}] Ref (kalman): {ref[:, k]}, Pos: {h[:, k]}, Error: {Error[:, k]}")
        u[:, k] = Controler_pos_PID(l, x[:, k], Error[:, k])
        send_velocity_control(u[:, k])
        rate.sleep()

    send_velocity_control([0, 0, 0, 0])

    np.save("PID_Kalman_vector.npy", x)
    np.save("PID_Kalman_error.npy", Error)
    np.save("PID_Kalman_h.npy", h[:, :-1])
    np.save("PID_Kalman_ref.npy", ref)

    plt.figure()
    plt.plot(t, h[0, :-1], label='x')
    plt.plot(t, h[1, :-1], label='y')
    plt.plot(t, h[2, :-1], label='z')
    plt.plot(t, ref[0], '--', label='x_ref')
    plt.plot(t, ref[1], '--', label='y_ref')
    plt.plot(t, ref[2], '--', label='z_ref')
    plt.title('Seguimiento del brazo - PID con Kalman')
    plt.xlabel('Tiempo [s]')
    plt.ylabel('Posición [m]')
    plt.legend()
    plt.grid()

    plt.figure()
    plt.plot(t, Error[0], label='Error x')
    plt.plot(t, Error[1], label='Error y')
    plt.plot(t, Error[2], label='Error z')
    plt.title('Error por componente - PID con Kalman')
    plt.xlabel('Tiempo [s]')
    plt.ylabel('Error [m]')
    plt.legend()
    plt.grid()
    plt.show()

    savemat("Control_Kin_Arm_4DOF_PID_Kalman.mat", {
        'h': h,
        'h_d': ref,
        't': t,
        'u': u,
        'x_e': Error,
        'q': x
    })

if __name__ == '__main__':
    try:
        rospy.init_node("Controlador_PID_Kalman", disable_signals=True, anonymous=True)
        rospy.Subscriber("/states", Joy, states_call_back)
        rospy.Subscriber("/aruco_position", Point, ARUCO_POSITION_call_back)
        main()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        send_velocity_control([0, 0, 0, 0])
        print("Interrupción detectada. Parando el brazo.")

