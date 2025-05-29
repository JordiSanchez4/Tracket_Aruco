import subprocess
import time

Tiempos = [100]
script = "PID_controller.py"  # Cambia esto por el nombre real de tu script PID

for T in Tiempos:
    print(f"=== Ejecutando PID con T_final={T} ===")
    subprocess.run(["python3", script, str(T)])
    print("→ Completado\n")
    time.sleep(2)

