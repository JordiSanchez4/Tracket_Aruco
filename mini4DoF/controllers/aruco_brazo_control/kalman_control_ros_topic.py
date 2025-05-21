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

# Variables globales para la cinemática del brazo
q1 = q2 = q3 = q4 = 0.0
q1_p = q2_p = q3_p = q4_p = 0.0

# Posición del ArUco (objetivo dinámico)
posicion_aruco = np.array([0.15, 0.15, 0.04])
aruco_lock = threading.Lock()

# === Filtro de Kalman ===
class KalmanFilter:
    def __init__(self, dt=0.02):
        self.x = np.zeros((6, 1))  # [x, y, z, vx, vy, vz]
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

def Controler_pos(L, q, he, hdp, val):
    J = jacobiana_Brazo4DOF(L, q)
    K = val*np.eye(3)
    qd = np.radians([0, 30, -15, 30])
    n = qd - np.array(q)
    D = np.diag([1, 5, 5, 10])
    TAREA_S = (np.eye(4) - np.linalg.pinv(J) @ J) @ (D @ n)
    Vref = np.linalg.pinv(J) @ (K @ np.tanh(0.5 * he)) + TAREA_S
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

    x[:, 0] = get_pose_arm()
    x_p[:, 0] = get_vel_arm()
    h[:, 0] = CDArm4DOF(l, x[:, 0])

    rate = rospy.Rate(frec)
    ref = np.zeros((3, t.shape[0]))
    ref_p = np.zeros((3, t.shape[0]))
    Error = np.zeros((3, t.shape[0]))
    K = 0.58

    kalman = KalmanFilter(dt=t_s)

    for k in range(t.shape[0]):
        with aruco_lock:
            ref[:, k] = kalman.update(posicion_aruco.copy())

        x[:, k] = get_pose_arm()
        x_p[:, k] = get_vel_arm()
        h[:, k] = CDArm4DOF(l, x[:, k])
        Error[:, k] = ref[:, k] - h[:, k]

        print(f"[{k}] Ref (kalman): {ref[:, k]}, Pos: {h[:, k]}, Error: {Error[:, k]}")
        u[:, k] = Controler_pos(l, x[:, k], Error[:, k], ref_p[:, k], K)
        send_velocity_control(u[:, k])
        rate.sleep()

    send_velocity_control([0, 0, 0, 0])

    np.save("MinNorm_Kalman_vector.npy", x)
    np.save("MinNorm_Kalman_error.npy", Error)
    np.save("MinNorm_Kalman_h.npy", h[:, :-1])
    np.save("MinNorm_Kalman_ref.npy", ref)

    plt.figure()
    plt.plot(t, h[0, :-1], label='x')
    plt.plot(t, h[1, :-1], label='y')
    plt.plot(t, h[2, :-1], label='z')
    plt.plot(t, ref[0], '--', label='x_ref')
    plt.plot(t, ref[1], '--', label='y_ref')
    plt.plot(t, ref[2], '--', label='z_ref')
    plt.title('Seguimiento del extremo del brazo (Kalman)')
    plt.xlabel('Tiempo [s]')
    plt.ylabel('Posición [m]')
    plt.legend()
    plt.grid()

    plt.figure()
    plt.plot(t, Error[0], label='Error x')
    plt.plot(t, Error[1], label='Error y')
    plt.plot(t, Error[2], label='Error z')
    plt.title('Error con Kalman Filter')
    plt.xlabel('Tiempo [s]')
    plt.ylabel('Error [m]')
    plt.legend()
    plt.grid()

    plt.show()

    savemat("Control_Kin_Arm_4DOF_Kalman.mat", {
        'h': h,
        'h_d': ref,
        't': t,
        'u': u,
        'x_e': Error,
        'q': x
    })

if __name__ == '__main__':
    try:
        rospy.init_node("Controlador_Kalman", disable_signals=True, anonymous=True)
        rospy.Subscriber("/states", Joy, states_call_back)
        rospy.Subscriber("/aruco_position", Point, ARUCO_POSITION_call_back)
        main()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        send_velocity_control([0, 0, 0, 0])
        print("\nInterrupción detectada.")

