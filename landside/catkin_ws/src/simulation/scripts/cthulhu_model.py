import numpy as np
import math

class Cthulhu:

    A = 0.0254  # conversion from inches to vrep units
    # MC note: pretty sure "vrep units" are meters. Could be wrong, but adjusted a to match
    QUADRATIC = True   # Set drag force to quadratic

    DYTOP = 7.69 * A    # same front and back
    DYBOTTOM = 6.47 * A
    DXTOPFRONT = 9.71 * A
    DXBOTTOMFRONT = 9.75 * A

    DXBACKTOP = -7.69 * A   # assume top is same as bottom
    DXBACKBOTTOM = -7.75 * A
    DZTOP = 1.17 * A
    # MC note: for stability. Replace with commented values for more accuracy
    # MC note: density assumed to be uniform? In any case, very little force rotates the bot

    DZBOTTOM = -5.44 * A

    THRUSTER_POINTS = (
        ( DXTOPFRONT, -DYTOP, DZTOP ),
        ( DXTOPFRONT, DYTOP, DZTOP ),
        ( DXBACKTOP, -DYTOP, DZTOP ),
        ( DXBACKTOP, DYTOP, DZTOP ),
        ( DXBOTTOMFRONT, -DYBOTTOM, DZBOTTOM ),
        ( DXBOTTOMFRONT, DYBOTTOM, DZBOTTOM ),
        ( DXBACKBOTTOM, -DYBOTTOM, DZBOTTOM ),
        ( DXBACKBOTTOM, DYBOTTOM, DZBOTTOM ),
    )

    P = 1000
    WATERLEVEL = 0

    THRUSTERFORCE= 5.25 # 4.1 rev
    ACTUALMASS = 22

    DRAG_COEF = 1.1 #original: 1.1

    # -4, 1, 3 from vincent
    #these numbers calc'd for a 15 deg pitch, and arbitrary roll (it looks about right)
    #centerOfBuoy = {-.1749*a, -.15*a * 0, 2*a}
    CENTER_OF_BUOY = ( 0, 0, 2 * A )
    CENTER_OF_MASS = ( .032, 0, .005 ) # currently unused

    def __init__(self, m):
       self.M = m
       self.FUDGEFORCE = self.THRUSTERFORCE * m / self.ACTUALMASS

    
    def get_mechanical_forces(self, xsize, ysize, zsize, grav, pos, v, angv):
        fbuoy = xsize * ysize
        pos.z -= zsize / 2.0
        subdepth = zsize if (zsize <= -pos.z) else -pos.z
        fbuoy *= subdepth  * -1 * grav.z * self.P
        if pos.z > 0:
            fbuoy = 0
            subdepth = 0

        dragforce = [
            self.calc_dragforcelin(v.x, ysize, subdepth),
            self.calc_dragforcelin(v.y, xsize, subdepth),
            self.calc_dragforcelin(v.z, xsize, ysize),
            self.calc_dragforceang(angv.x, ysize, xsize),
            self.calc_dragforceang(angv.y, ysize, xsize),
            self.calc_dragforceang(angv.z, ysize, subdepth)
        ]
        if pos.z > 0:
            dragforce[2] = 0
        print(dragforce)
        return [0, 0, fbuoy], dragforce

    def calc_dragforcelin(self, linvel, length, depth):
        if self.QUADRATIC:
            return -self.P * (linvel ** 2) * np.sign(linvel) * self.DRAG_COEF * length * depth
        return -self.P * abs(linvel) * np.sign(linvel) * self.DRAG_COEF * length * depth
    
    def calc_dragforceang(self, angvel, length, depth):
        angdragfudgecoef = 1  # 0.05
        if self.QUADRATIC:
            return -self.P * (angvel ** 2) * np.sign(angvel) * self.DRAG_COEF * (length ** 3) * depth / 12 * angdragfudgecoef
        return -self.P * abs(angvel) * np.sign(angvel) * self.DRAG_COEF * (length ** 2) * depth / 4 * angdragfudgecoef

    def flip_thrusters(self, forcesloc):
        flipped = [True, True, False, True, False, True, True, False]
        for i in range(len(flipped)):
            if flipped[i]:
                for k in range(3):
                    forcesloc[i][k] = -1 * forcesloc[i][k]
        return forcesloc

    def get_thruster_forces(self, vals):
        if vals is None:
            return None
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
                    forces[count][num] = (vals[count]) * ((1 / math.sqrt(2)) * self.FUDGEFORCE * (forces[count][num]))
                else:
                    forces[count][num] = (vals[count]) * self.FUDGEFORCE * (forces[count][num])
        return forces
    
