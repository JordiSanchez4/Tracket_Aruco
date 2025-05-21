import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# === Cargar archivos de error y posición ===
pid_error = np.load("PID_error.npy")
minnorm_error = np.load("MinNorm_error.npy")
pid_pos = np.load("PID_vector.npy")
minnorm_pos = np.load("MinNorm_vector.npy")

# === Cargar posiciones reales y referencias
pid_h = np.load("PID_h.npy")
pid_ref = np.load("PID_ref.npy")
min_h = np.load("MinNorm_h.npy")
min_ref = np.load("MinNorm_ref.npy")

# === Igualar duración (por seguridad)
min_len = min(pid_error.shape[1], minnorm_error.shape[1],
              pid_h.shape[1], min_h.shape[1], pid_ref.shape[1], min_ref.shape[1])
frec = 50
t = np.arange(0, min_len) / frec

# Recortar todos
pid_error = pid_error[:, :min_len]
minnorm_error = minnorm_error[:, :min_len]
pid_h = pid_h[:, :min_len]
min_h = min_h[:, :min_len]
pid_ref = pid_ref[:, :min_len]
min_ref = min_ref[:, :min_len]

# === Función RMSE
def compute_rmse(error):
    rmse_x = np.sqrt(np.mean(error[0]**2))
    rmse_y = np.sqrt(np.mean(error[1]**2))
    rmse_z = np.sqrt(np.mean(error[2]**2))
    rmse_total = np.sqrt(np.mean(np.sum(error**2, axis=0)))
    return rmse_x, rmse_y, rmse_z, rmse_total

# === Calcular RMSE
rmse_pid = compute_rmse(pid_error)
rmse_min = compute_rmse(minnorm_error)

# === Guardar tabla RMSE
rmse_df = pd.DataFrame({
    "Controlador": ["PID", "Norma Mínima"],
    "RMSE X [m]": [rmse_pid[0], rmse_min[0]],
    "RMSE Y [m]": [rmse_pid[1], rmse_min[1]],
    "RMSE Z [m]": [rmse_pid[2], rmse_min[2]],
    "RMSE Total [m]": [rmse_pid[3], rmse_min[3]]
})
print("\n========== TABLA DE RMSE ==========\n")
print(rmse_df)
rmse_df.to_csv("rmse_comparacion.csv", index=False)

# === Gráfico: norma del error cartesiano
plt.figure()
plt.plot(t, np.linalg.norm(pid_error, axis=0), label='PID')
plt.plot(t, np.linalg.norm(minnorm_error, axis=0), label='Norma Mínima')
plt.title("Comparación de la norma del error cartesiano")
plt.xlabel("Tiempo [s]")
plt.ylabel("||Error|| [m]")
plt.legend()
plt.grid()
plt.tight_layout()
plt.savefig("comparacion_error_norma.png")

# === Gráfico: errores por eje
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

# === Gráfico: seguimiento de referencia vs posición real
for i in range(3):
    plt.figure()
    plt.plot(t, pid_h[i], label='PID')
    plt.plot(t, min_h[i], label='Norma Mínima')
    plt.plot(t, pid_ref[i], 'k--', label='Referencia (ArUco)')
    plt.title(f"Seguimiento en eje {ejes[i]}")
    plt.xlabel("Tiempo [s]")
    plt.ylabel(f'{ejes[i]} [m]')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.savefig(f"comparacion_tracking_{ejes[i]}.png")

print("\n✅ Comparación completada. Archivos generados:")
print("- rmse_comparacion.csv")
print("- comparacion_error_norma.png")
print("- comparacion_errores_xyz.png")
print("- comparacion_tracking_x/y/z.png")

