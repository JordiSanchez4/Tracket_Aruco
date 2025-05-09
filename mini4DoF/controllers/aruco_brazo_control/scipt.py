from scipy.io import loadmat

data = loadmat(r"/home/artil/Course_Robot_Arm_4DOF/Control_Kin_Arm_4DOF.mat")

print(data.keys())