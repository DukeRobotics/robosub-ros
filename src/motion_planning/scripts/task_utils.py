import numpy as np

class TaskUtils(object):

    def distance(self, x1, y1, z1, x2, y2, z2):
        vector1 = np.array([x1, y1, z1])
        vector2 = np.array([x2, y2, z2])
        distance = np.linalg.norm(vector2 - vector1)
        return distance

    def transform(self):
