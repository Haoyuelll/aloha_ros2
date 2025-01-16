import numpy as np
from scipy.spatial.transform import Rotation

world_from_robot = np.array([[1, 0, 0, -0.449],
                             [0, 1, 0, -0.019],
                             [0, 0, 1, 0.02],
                             [0, 0, 0, 1]])

data = np.load('T_base2camera_left_arm202501151839.npy')

result = np.matmul(world_from_robot, data)

r = Rotation.from_matrix(result[:3][:3]) 
quat = r.as_quat()