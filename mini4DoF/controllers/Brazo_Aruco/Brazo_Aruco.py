# Brazo_Aruco.py corregido para seguir al ArUco usando 4 motores en Webots + ROS1

import math
import rospy
from sensor_msgs.msg import Joy
from controller import Robot

# Constantes
MAX_VELOCITY = 2.5  # rad/s (ajustar según tu motor)
CONTROL_GAIN = 0.7  # Ganancia proporcional de control

# Variables de control recibidas
desired_u1 = 0.0
desired_u2 = 0.0
desired_u3 = 0.0
desired_u4 = 0.0

def control_callback(msg):
    global desired_u1, desired_u2, desired_u3, desired_u4
    desired_u1 = msg.axes[0]
    desired_u2 = msg.axes[1]
    desired_u3 = msg.axes[2]
    desired_u4 = msg.axes[3]

def saturate(value, limit):
    return max(min(value, limit), -limit)

def main():
    # Inicializar Webots
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    # Inicializar ROS
    rospy.init_node('brazo_aruco_webots', anonymous=True, disable_signals=True)
    rospy.Subscriber('/control', Joy, control_callback)
    rate = rospy.Rate(30)

    # Obtener motores
    motors = [
        robot.getDevice('m1_continuous'),
        robot.getDevice('m2'),
        robot.getDevice('m3'),
        robot.getDevice('m4')
    ]
    
    for motor in motors:
        motor.setPosition(float('inf'))
        motor.setVelocity(0.0)

    # Obtener sensores de posición
    sensors = [
        robot.getDevice('m1_continuous_sensor'),
        robot.getDevice('m2_sensor'),
        robot.getDevice('m3_sensor'),
        robot.getDevice('m4_sensor')
    ]

    for sensor in sensors:
        sensor.enable(timestep)

    print("[INFO] Brazo_Aruco.py iniciado correctamente.")

    while robot.step(timestep) != -1 and not rospy.is_shutdown():
        # Leer posiciones actuales
        current_positions = [sensor.getValue() for sensor in sensors]
        
        # Aplicar control proporcional simple
        vel_m1 = saturate(CONTROL_GAIN * desired_u1, MAX_VELOCITY)
        vel_m2 = saturate(CONTROL_GAIN * desired_u2, MAX_VELOCITY)
        vel_m3 = saturate(CONTROL_GAIN * desired_u3, MAX_VELOCITY)
        vel_m4 = saturate(CONTROL_GAIN * desired_u4, MAX_VELOCITY)

        # Comandar velocidades
        motors[0].setVelocity(vel_m1)
        motors[1].setVelocity(vel_m2)
        motors[2].setVelocity(vel_m3)
        motors[3].setVelocity(vel_m4)

        # Debug info
        print("[INFO] Actual (grados): M1: {:.2f} | M2: {:.2f} | M3: {:.2f} | M4: {:.2f}".format(
            math.degrees(current_positions[0]),
            math.degrees(current_positions[1]),
            math.degrees(current_positions[2]),
            math.degrees(current_positions[3])
        ))
        print("[INFO] Control Vel: U1: {:.2f} | U2: {:.2f} | U3: {:.2f} | U4: {:.2f}".format(
            vel_m1, vel_m2, vel_m3, vel_m4
        ))

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("[INFO] Brazo_Aruco.py interrumpido.")

