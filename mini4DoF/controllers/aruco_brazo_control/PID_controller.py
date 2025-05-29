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
    # PID gains
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
    import sys
    t_final = 200
    #t_final = int(sys.argv[1]) if len(sys.argv) > 1 else 200

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

    for k in range(t.shape[0]):
        with aruco_lock:
            ref[:, k] = posicion_aruco.copy()

        x[:, k] = get_pose_arm()
        x_p[:, k] = get_vel_arm()
        h[:, k] = CDArm4DOF(l, x[:, k])
        Error[:, k] = ref[:, k] - h[:, k]

        print(f"[{k}] Ref: {ref[:, k]}, Pos: {h[:, k]}, Error: {Error[:, k]}")

        u[:, k] = Controler_pos_PID(l, x[:, k], Error[:, k])
        send_velocity_control(u[:, k])
        rate.sleep()

    send_velocity_control([0, 0, 0, 0])

    # === Calcular RMSE por eje ===
    rmse_x = np.sqrt(np.mean(Error[0]**2))
    rmse_y = np.sqrt(np.mean(Error[1]**2))
    rmse_z = np.sqrt(np.mean(Error[2]**2))

    print(f"[DONE] RMSE_X={rmse_x:.4f}, RMSE_Y={rmse_y:.4f}, RMSE_Z={rmse_z:.4f}")

    # Guardar RMSE como archivo .npy
    np.save(f"RMSE_PID_T{t_final}.npy", np.array([rmse_x, rmse_y, rmse_z]))

    # Guardar resumen en .txt
    with open("Resultados_RMSE_PID.txt", "a") as f:
        f.write(f"T={t_final}s -> RMSE_X={rmse_x:.4f}, RMSE_Y={rmse_y:.4f}, RMSE_Z={rmse_z:.4f}\n")











    #np.savetxt("control.txt", x)
    #np.save("vector.npy", x)
    #np.save("vector12.npz", x, Error)
    np.save("PID_vector.npy", x)
    np.save("PID_error.npy", Error)
    np.save("PID_h.npy", h[:, :-1])
    np.save("PID_ref.npy", ref)


    # Plots
    plt.figure()
    plt.plot(t, h[0, :-1], label='x')
    plt.plot(t, h[1, :-1], label='y')
    plt.plot(t, h[2, :-1], label='z')
    plt.plot(t, ref[0], '--', label='x_ref')
    plt.plot(t, ref[1], '--', label='y_ref')
    plt.plot(t, ref[2], '--', label='z_ref')
    plt.title('Posición del extremo del brazo vs Referencia')
    plt.xlabel('Tiempo [s]')
    plt.ylabel('Posición [m]')
    plt.legend()
    plt.grid()

    plt.figure()
    plt.plot(t, Error[0], label='Error x')
    plt.plot(t, Error[1], label='Error y')
    plt.plot(t, Error[2], label='Error z')
    plt.title('Error por Eje')
    plt.xlabel('Tiempo [s]')
    plt.ylabel('Error [m]')
    plt.legend()
    plt.grid()
    plt.show()


    



    savemat("Control_Kin_Arm_4DOF_PID.mat", {
        'h': h,
        'h_d': ref,
        't': t,
        'u': u,
        'x_e': Error,
        'q': x
    })

if __name__ == '__main__':
    try:
        rospy.init_node("Controlador_PID", disable_signals=True, anonymous=True)
        rospy.Subscriber("/states", Joy, states_call_back)
        rospy.Subscriber("/aruco_position", Point, ARUCO_POSITION_call_back)
        main()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        send_velocity_control([0, 0, 0, 0])
        print("Interrupción detectada. Parando el brazo.")

