import numpy as np
from jacobiana_brazo_4dof import jacobiana_brazo_4dof

def controlador_minima_norma(l2, l3, l4, q, he, hdp):
    q1, q2, q3, q4 = q
    J = jacobiana_brazo_4dof(l2, l3, l4, q1, q2, q3, q4)
    K = np.eye(3)  # Ganancia
    return np.linalg.pinv(J).dot(hdp + K.dot(np.tanh(0.5 * he)))
