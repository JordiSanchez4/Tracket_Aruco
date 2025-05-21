import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

frec = 50

# === Cargar datos ===
error_pid = np.load("PID_error.npy")
error_pk = np.load("PID_Kalman_error.npy")
h_pid = np.load("PID_h.npy")
h_pk = np.load("PID_Kalman_h.npy")
ref = np.load("MinNorm_ref.npy")  # Usar referencia común

# === Alinear duraciones ===
min_len = min(error_pid.shape[1], error_pk.shape[1], h_pid.shape[1], h_pk.shape[1], ref.shape[1])
t = np.arange(0, min_len) / frec

# Recorte
error_pid = error_pid[:, :min_len]
error_pk = error_pk[:, :min_len]
h_pid = h_pid[:, :min_len]
h_pk = h_pk[:, :min_len]
ref = ref[:, :min_len]

# === Recalcular error real respecto a misma referencia
error_pid = ref - h_pid
error_pk = ref - h_pk

def compute_rmse(error):
    return [np.sqrt(np.mean(error[i]**2)) for i in range(3)] + [np.sqrt(np.mean(np.sum(error**2, axis=0)))]

rmse_pid = compute_rmse(error_pid)
rmse_pk = compute_rmse(error_pk)

rmse_df = pd.DataFrame({
    "Controlador": ["PID", "PID + Kalman"],
    "RMSE X [m]": [rmse_pid[0], rmse_pk[0]],
    "RMSE Y [m]": [rmse_pid[1], rmse_pk[1]],
    "RMSE Z [m]": [rmse_pid[2], rmse_pk[2]],
    "RMSE Total [m]": [rmse_pid[3], rmse_pk[3]]
})
print("\n========== RMSE PID vs PID+KF ==========\n")
print(rmse_df)
rmse_df.to_csv("rmse_pid_vs_pid_kf.csv", index=False)

# === Seguimiento
ejes = ['x', 'y', 'z']
for i in range(3):
    plt.figure()
    plt.plot(t, h_pid[i], label="PID")
    plt.plot(t, h_pk[i], label="PID + Kalman")
    plt.plot(t, ref[i], 'k--', label="Referencia")
    plt.title(f"Seguimiento - PID vs PID+KF - eje {ejes[i]}")
    plt.xlabel("Tiempo [s]")
    plt.ylabel(f"Posición {ejes[i]} [m]")
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.savefig(f"seguimiento_pid_vs_pid_kf_{ejes[i]}.png")

