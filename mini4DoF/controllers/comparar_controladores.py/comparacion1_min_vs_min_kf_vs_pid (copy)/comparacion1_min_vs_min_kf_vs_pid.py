import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

frec = 50
# === Cargar datos ===
error_min = np.load("MinNorm_error.npy")
error_mk = np.load("MinNorm_Kalman_error.npy")
h_min = np.load("MinNorm_h.npy")
h_mk = np.load("MinNorm_Kalman_h.npy")
ref = np.load("MinNorm_ref.npy")

min_len = min(error_min.shape[1], error_mk.shape[1], h_min.shape[1], h_mk.shape[1], ref.shape[1])
t = np.arange(0, min_len) / frec

error_min = error_min[:, :min_len]
error_mk = error_mk[:, :min_len]
h_min = h_min[:, :min_len]
h_mk = h_mk[:, :min_len]
ref = ref[:, :min_len]

def compute_rmse(error):
    return [np.sqrt(np.mean(error[i]**2)) for i in range(3)] + [np.sqrt(np.mean(np.sum(error**2, axis=0)))]

rmse_min = compute_rmse(error_min)
rmse_mk = compute_rmse(error_mk)

rmse_df = pd.DataFrame({
    "Controlador": ["MinNorm", "MinNorm + Kalman"],
    "RMSE X [m]": [rmse_min[0], rmse_mk[0]],
    "RMSE Y [m]": [rmse_min[1], rmse_mk[1]],
    "RMSE Z [m]": [rmse_min[2], rmse_mk[2]],
    "RMSE Total [m]": [rmse_min[3], rmse_mk[3]]
})
print("\n========== RMSE MinNorm vs MinNorm+KF ==========\n")
print(rmse_df)
rmse_df.to_csv("rmse_min_vs_min_kf.csv", index=False)

# === Seguimiento
ejes = ['x', 'y', 'z']
for i in range(3):
    plt.figure()
    plt.plot(t, h_min[i], label="MinNorm")
    plt.plot(t, h_mk[i], label="MinNorm + Kalman")
    plt.plot(t, ref[i], 'k--', label="Referencia")
    plt.title(f"Seguimiento - MinNorm vs MinNorm+KF - eje {ejes[i]}")
    plt.xlabel("Tiempo [s]")
    plt.ylabel(f"Posición {ejes[i]} [m]")
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.savefig(f"seguimiento_min_vs_min_kf_{ejes[i]}.png")

