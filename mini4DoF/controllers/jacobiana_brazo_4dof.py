import numpy as np

def jacobiana_brazo_4dof(l2, l3, l4, q1, q2, q3, q4):
    J = np.array([
        [-np.sin(q1)*(l3*np.cos(q2 + q3) + l2*np.cos(q2) + l4*np.cos(q2 + q3 + q4)), -np.cos(q1)*(l3*np.sin(q2 + q3) + l2*np.sin(q2) + l4*np.sin(q2 + q3 + q4)), -np.cos(q1)*(l3*np.sin(q2 + q3) + l4*np.sin(q2 + q3 + q4)), -l4*np.sin(q2 + q3 + q4)*np.cos(q1)],
        [ np.cos(q1)*(l3*np.cos(q2 + q3) + l2*np.cos(q2) + l4*np.cos(q2 + q3 + q4)), -np.sin(q1)*(l3*np.sin(q2 + q3) + l2*np.sin(q2) + l4*np.sin(q2 + q3 + q4)), -np.sin(q1)*(l3*np.sin(q2 + q3) + l4*np.sin(q2 + q3 + q4)), -l4*np.sin(q2 + q3 + q4)*np.sin(q1)],
        [0, l3*np.cos(q2 + q3) + l2*np.cos(q2) + l4*np.cos(q2 + q3 + q4), l3*np.cos(q2 + q3) + l4*np.cos(q2 + q3 + q4), l4*np.cos(q2 + q3 + q4)]
    ])
    return J
