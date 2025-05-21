import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# === Cargar archivos guardados ===
pid_error = np.load("PID_error.npy")
minnorm_error = np.load("MinNorm_error.npy")
pid_pos = np.load("PID_vector.npy")
minnorm_pos = np.load("MinNorm_vector.npy")

# === Asegurar duración igualada ===
min_len = min(pid_error.shape[1], minnorm_error.shape[1])
pid_error = pid_error[:, :min_len]
minnorm_error = minnorm_error[:, :min_len]
t = np.arange(0, min_len) / 50.0  # frecuencia 50 Hz

# === RMSE function ===
def compute_rmse(error):
    return np.sqrt(np.mean(error[0]**2)), np.sqrt(np.mean(error[1]**2)), np.sqrt(np.mean(error[2]**2)), np.sqrt(np.mean(np.sum(error**2, axis=0)))

rmse_pid = compute_rmse(pid_error)
rmse_min = compute_rmse(minnorm_error)

# === Tabla comparativa ===
rmse_df = pd.DataFrame({
    "Controlador": ["PID", "Norma Mínima"],
    "RMSE X [m]": [rmse_pid[0], rmse_min[0]],
    "RMSE Y [m]": [rmse_pid[1], rmse_min[1]],
    "RMSE Z [m]": [rmse_pid[2], rmse_min[2]],
    "RMSE Total [m]": [rmse_pid[3], rmse_min[3]]
})
print(rmse_df)

# === Gráfico: error cartesiano normado ===
plt.figure()
plt.plot(t, np.linalg.norm(pid_error, axis=0), label='PID')
plt.plot(t, np.linalg.norm(minnorm_error, axis=0), label='Norma Mínima')
plt.title("Norma del Error cartesiano ||Error||")
plt.xlabel("Tiempo [s]")
plt.ylabel("Error [m]")
plt.legend()
plt.grid()
plt.tight_layout()
plt.savefig("comparacion_error_norma.png")

# === Gráfico: errores por eje ===
fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
ejes = ['x', 'y', 'z']
for i in range(3):
    axs[i].plot(t, pid_error[i], label='PID')
    axs[i].plot(t, minnorm_error[i], label='Norma Mínima')
    axs[i].set_ylabel(f'Error {ejes[i]} [m]')
    axs[i].legend()
    axs[i].grid()
axs[2].set_xlabel("Tiempo [s]")
plt.suptitle("Errores por componente cartesiano")
plt.tight_layout()
plt.savefig("comparacion_errores_xyz.png")

# === Guardar RMSE como tabla TXT también ===
with open("rmse_comparacion.txt", "w") as f:
    f.write(rmse_df.to_string(index=False))

