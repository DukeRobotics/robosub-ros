import numpy as np

def distance(self, x1, y1, z1, x2, y2, z2):
    vector1 = np.array([x1, y1, z1])
    vector2 = np.array([x2, y2, z2])
    distance = np.linalg.norm(vector2 - vector1)
    return distance

def linear_distance(point1, point2):
    vector1 = np.array([point1.x, point1.y, point1.z])
    vector2 = np.array([point2.x, point2.y, point2.z])
    distance = np.linalg.norm(vector2 - vector1)
    return distance

def angular_distance_quat(quat1, quat2):
    """
    """
    #convert to rpy, call rpy method
    pass

def angular_distance_rpy(rpy1, rpy2):
    """
    """
    pass

def at_pose(current_pose, desired_pose, linear_tol=0.1, angular_tol=3):
    """
    """
    pass

def transform(self):
    """
    """
    pass
