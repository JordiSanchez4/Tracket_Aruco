import numpy as np
import matplotlib.pyplot as plt

frec = 50
h_pid = np.load("PID_h.npy")
h_pk = np.load("PID_Kalman_h.npy")
ref = np.load("MinNorm_ref.npy")  # misma referencia usada para alinear todo

min_len = min(h_pid.shape[1], h_pk.shape[1], ref.shape[1])
t = np.arange(0, min_len) / frec

h_pid = h_pid[:, :min_len]
h_pk = h_pk[:, :min_len]
ref = ref[:, :min_len]

ejes = ['x', 'y', 'z']
for i in range(3):
    plt.figure()
    plt.plot(t, h_pid[i], label="PID")
    plt.plot(t, h_pk[i], label="PID + Kalman")
    plt.plot(t, ref[i], '--k', label="Referencia")
    plt.title(f'PID vs PID+KF - eje {ejes[i]}')
    plt.xlabel("Tiempo [s]")
    plt.ylabel(f'Posición {ejes[i]} [m]')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.savefig(f"comparacion_pid_vs_pid_kf_{ejes[i]}.png")

