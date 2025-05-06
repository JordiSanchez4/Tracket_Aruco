from controller import Supervisor
import math

supervisor = Supervisor()
moving_point = supervisor.getFromDef("MovingPoint")

timestep = int(supervisor.getBasicTimeStep())
radius = 0.1  # Radio del círculo
speed = 0.5    # Velocidad angular en rad/s
center_x, center_y = 2.8, 3.0  # Centro del movimiento

time = 0  # Inicializa el tiempo

while supervisor.step(timestep) != -1:
    time += timestep / 1000  # Convierte a segundos
    new_x = center_x + radius * math.cos(speed * time)
    new_y = center_y + radius * math.sin(speed * time)
    
    moving_point.getField("translation").setSFVec3f([new_x, new_y, 0.78])

    print(f"MovingPoint position: {new_x}, {new_y}, 0.78")

