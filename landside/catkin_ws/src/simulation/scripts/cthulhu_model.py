import numpy as np
import math

class Cthulhu:

    THRUSTERFORCE= 5.25 # 4.1 rev
    ACTUALMASS = 22

    def __init__(self, m):
       self.M = m
       self.FUDGEFORCE = self.THRUSTERFORCE * m / self.ACTUALMASS

    def flip_thrusters(self, forcesloc):
        flipped = [True, True, False, True, False, True, True, False]
        for i in range(len(flipped)):
            if flipped[i]:
                for k in range(3):
                    forcesloc[i][k] = -1 * forcesloc[i][k]
        return forcesloc

    def get_thruster_forces(self, vals):
        clipped = np.clip(np.array(vals) / 127, -1, 1)
        forces = [
            [ 1, 1, 0 ], #tfr
            [ 1, -1, 0 ], #tfl
            [ -1, 1, 0 ], #tbr
            [ -1, -1, 0 ], #tbl
            [ 0, 0, -1 ],
            [ 0, 0, -1 ],
            [ 0, 0, -1 ],
            [ 0, 0, -1 ],
        ]
        forces = self.flip_thrusters(forces)

        for count in range(len(forces)):
            for num in range(3):
                if count < 4:
                    forces[count][num] = (clipped[count]) * ((1 / math.sqrt(2)) * self.FUDGEFORCE * (forces[count][num]))
                else:
                    forces[count][num] = (clipped[count]) * self.FUDGEFORCE * (forces[count][num])
        return forces
