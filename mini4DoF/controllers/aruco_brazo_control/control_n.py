import rospy
from sensor_msgs.msg import Joy
import numpy as np
from scipy.io import savemat
import os

# Características del Brazo
l = [0.0676, 0.06883, 0.06883, 0.15916]

# Variables globales para la cinemática del brazo
q1 = q2 = q3 = q4 = 0.0
q1_p = q2_p = q3_p = q4_p = 0.0

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

def get_pose_arm():
    return [q1, q2, q3, q4]

def get_vel_arm():
    return [q1_p, q2_p, q3_p, q4_p]

def send_velocity_control(u):
    control_msg = Joy()
    control_pub = rospy.Publisher("/control", Joy, queue_size=10)
    control_msg.header.frame_id = "base_link"
    control_msg.header.stamp = rospy.Time.now()
    control_msg.axes = u
    control_pub.publish(control_msg)

def main():
    t_final = 60*4
    frec = 30
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
    K = 0.5

    # Lista de posiciones de referencia (más puntos cercanos)
    ref_list = [
        np.array([0.15, 0.0, 0.04]),
        np.array([0.0, -0.20, 0.04]),
        np.array([-0.16, 0.00, 0.06]),
        np.array([0.00, 0.15, 0.05]),
        np.array([0.14, 0.12, 0.07])
    ]

    ref_idx = 0
    umbral = 0.01

    for k in range(t.shape[0]):
        ref[:, k] = ref_list[ref_idx]
        x[:, k] = get_pose_arm()
        x_p[:, k] = get_vel_arm()
        h[:, k] = CDArm4DOF(l, x[:, k])
        Error[:, k] = ref[:, k] - h[:, k]

        print(f"Target {ref_idx + 1}/{len(ref_list)}, Error: {Error[:, k]}")

        if np.linalg.norm(Error[:, k]) < umbral:
            if ref_idx < len(ref_list) - 1:
                ref_idx += 1

        u[:, k] = Controler_pos(l, x[:, k], Error[:, k], ref_p[:, k], K)
        send_velocity_control(u[:, k])
        rate.sleep()

    send_velocity_control([0, 0, 0, 0])

    pwd = "/home/bryansgue/Doctoral_Research/Cursos/Course_Robot_Arm_4DOF/Matlab"
    if not os.path.exists(pwd):
        print(f"La ruta {pwd} no existe. Estableciendo la ruta local como pwd.")
        pwd = os.getcwd()

    savemat(os.path.join(pwd, "Control_Kin_Arm_4DOF.mat"), {
        'h': h,
        'h_d': ref,
        't': t,
        'u': u,
        'x_e': Error,
        'q': x
    })


if __name__ == '__main__':
    try:
        rospy.init_node("Controlador", disable_signals=True, anonymous=True)
        rospy.Subscriber("/states", Joy, states_call_back)
        main()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        print("\nError System")
        send_velocity_control([0, 0, 0, 0])
    else:
        print("Complete Execution")
