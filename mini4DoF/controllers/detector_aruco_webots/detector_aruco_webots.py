from controller import Robot
import numpy as np
import cv2
import cv2.aruco as aruco
import rospy
from geometry_msgs.msg import Point

robot = Robot()
timestep = int(robot.getBasicTimeStep())
cam = robot.getDevice("WebCam")
cam.enable(timestep)

rospy.init_node("aruco_webots_detector", disable_signals=True)
pub = rospy.Publisher("/aruco_position", Point, queue_size=1)

# Matriz de calibración (puedes refinarla más tarde con calibración realista)
camera_matrix = np.array([[800, 0, 640],
                          [0, 800, 360],
                          [0,   0,   1]], dtype=np.float32)
dist_coeffs = np.zeros((5, 1))  # Webots no tiene distorsión por defecto
marker_length = 0.07  # Tamaño real del marcador en metros

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
aruco_params = aruco.DetectorParameters_create()

while robot.step(timestep) != -1 and not rospy.is_shutdown():
    width = cam.getWidth()
    height = cam.getHeight()
    raw_image = cam.getImage()
    
    img = np.frombuffer(raw_image, np.uint8).reshape((height, width, 4))
    frame = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

    corners, ids, _ = aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)
    
    if ids is not None:
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
        for i, id in enumerate(ids):
            if id[0] == 1:
                aruco.drawDetectedMarkers(frame, corners)
                aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.03)
                t = tvecs[i][0]  # posición [x, y, z] en metros
                pub.publish(Point(x=t[0], y=t[1], z=t[2]))
                rospy.loginfo(f"[CAMARA] ArUco ID 1 -> Posición detectada: {t}")
    
    cv2.imshow("Webots Camera ArUco", frame)
    cv2.waitKey(1)
