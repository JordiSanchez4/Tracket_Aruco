import cv2
import cv2.aruco as aruco
import numpy as np

# === Camera Intrinsic Parameters ===
# Camera intrinsic matrix (3x3)
camera_matrix = np.array([
    [3457.25272,     0.0,         319.59303],
    [0.0,         3602.16792,     241.572232],
    [0.0,            0.0,            1.0]
], dtype=np.float64)

# Distortion coefficients (k1, k2, p1, p2, k3)
dist_coeffs = np.array([
    1.68333529,    # k1
   -93.3657854,    # k2
   -0.0110155886,  # p1
   -0.0333554301,  # p2
   -0.516887297    # k3
], dtype=np.float64)

# === Marker size in meters ===
marker_length = 0.0088

# === ArUco dictionary and parameters ===
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

def get_z_and_inclination_from_single_frame():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open webcam.")
        return None, None

    ret, frame = cap.read()
    cap.release()

    if not ret:
        print("Failed to capture frame.")
        return None, None

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    z_values = []
    angles = []

    if ids is not None:
        for i in range(len(ids)):
            retval, rvec, tvec = cv2.solvePnP(
                objectPoints=np.array([
                    [-marker_length / 2,  marker_length / 2, 0],
                    [ marker_length / 2,  marker_length / 2, 0],
                    [ marker_length / 2, -marker_length / 2, 0],
                    [-marker_length / 2, -marker_length / 2, 0]
                ], dtype=np.float32),
                imagePoints=corners[i][0],
                cameraMatrix=camera_matrix,
                distCoeffs=dist_coeffs
            )

            if retval:
                # Z-distance
                z = tvec[2][0]
                z_values.append(z)

                # Inclination calculation
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                marker_z_axis = rotation_matrix[:, 2]
                cos_theta = marker_z_axis[2]
                cos_theta = np.clip(cos_theta, -1.0, 1.0)  # Numerical stability
                angle_rad = np.arccos(cos_theta)
                angle_deg = np.degrees(angle_rad)
                angles.append(angle_deg)

    if z_values and angles:
        avg_z = np.mean(z_values)
        avg_angle = np.mean(angles)
        print(f"[INFO] Avg Z-distance: {avg_z:.3f} m | Avg Inclination: {avg_angle:.2f}°")
        return avg_z, avg_angle
    else:
        print("[WARN] No markers detected.")
        return None, None
