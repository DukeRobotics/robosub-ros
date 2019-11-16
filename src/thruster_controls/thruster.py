import rospy
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Vector3
import math

class Thruster():

    # Constants for scaling thruster speeds to ROS thruster controller
    MAX_NEG_POW = -128
    MAX_POS_POW = 127

    def __init__(self, pos, rpy):
        self.pos = pos
        self.rpy = rpy
        
        to_rad = lambda x: x * math.pi / 180  # convert degrees to radians
        q = quaternion_from_euler(to_rad(rpy[0]), to_rad(rpy[1]), to_rad(rpy[2]))

        self.force_hat = q * Vector3(1, 0, 0)
        
        pos_vec3 = Vector3(self.pos[0], self.pos[1], self.pos[2])
        self.torque = pos_vec3.cross(self.force_hat)
