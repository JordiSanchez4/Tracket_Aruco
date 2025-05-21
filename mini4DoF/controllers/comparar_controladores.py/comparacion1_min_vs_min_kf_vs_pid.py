import numpy as np
import matplotlib.pyplot as plt

frec = 50
h_min = np.load("MinNorm_h.npy")
h_mk = np.load("MinNorm_Kalman_h.npy")
h_pid = np.load("PID_h.npy")
ref = np.load("MinNorm_ref.npy")

min_len = min(h_min.shape[1], h_mk.shape[1], h_pid.shape[1], ref.shape[1])
t = np.arange(0, min_len) / frec

h_min = h_min[:, :min_len]
h_mk = h_mk[:, :min_len]
h_pid = h_pid[:, :min_len]
ref = ref[:, :min_len]

ejes = ['x', 'y', 'z']
for i in range(3):
    plt.figure()
    plt.plot(t, h_min[i], label="MinNorm")
    plt.plot(t, h_mk[i], label="MinNorm+Kalman")
    plt.plot(t, h_pid[i], label="PID")
    plt.plot(t, ref[i], '--k', label="Referencia")
    plt.title(f'Comparación 1 - Seguimiento eje {ejes[i]}')
    plt.xlabel("Tiempo [s]")
    plt.ylabel(f'Posición {ejes[i]} [m]')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.savefig(f"comparacion1_tracking_{ejes[i]}.png")

