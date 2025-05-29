import subprocess
import time

# Lista de Ks y tiempos a probar
Ks = [1]
Tiempos = [50]

# Ruta del script del controlador
script = "control_ros_topic.py"

# Ejecutar cada combinación
for K in Ks:
    for T in Tiempos:
        print(f"=== Ejecutando K={K}, T_final={T} ===")
        subprocess.run(["python3", script, str(K), str(T)])
        print("→ Completado\n")
        time.sleep(2)  # pequeño delay por estabilidad ROS/Webots

