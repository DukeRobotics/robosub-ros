import sim
import rospy
import sys
import traceback
from geometry_msgs.msg import Pose, Quaternion, Point, Twist, Vector3
import numpy as np
import itertools

class SimHandle:

    DOCKER_IP = '192.168.65.2'

    def __init__(self):
        sim.simxFinish(-1)
        self.clientID = sim.simxStart(self.DOCKER_IP, 8080, True, True, 5000, 5)
        if self.clientID == -1:
            rospy.logerr('Failed connecting to remote API server')
            sim.simxFinish(-1)
            sys.exit(1)
        rospy.loginfo('Connected to remote API server')
        rospy.loginfo('Testing connection')
        objs = self.run_sim_function(
            sim.simxGetObjects, (self.clientID, sim.sim_handle_all, sim.simx_opmode_blocking))
        rospy.loginfo(f'Number of objects in the scene: {len(objs)}')
        self.robot = self.run_sim_function(
            sim.simxGetObjectHandle, (self.clientID, "Rob", sim.simx_opmode_blocking))
        self.set_position_to_zero()
        rospy.sleep(0.1)
        self.init_streaming()
        rospy.loginfo("Starting main loop")

    def init_streaming(self):
        self.get_pose(mode=sim.simx_opmode_streaming)
        self.get_twist(mode=sim.simx_opmode_streaming)
        #self.get_gravity(mode=sim.simx_opmode_streaming)
        #self.get_size(mode=sim.simx_opmode_streaming)

    def run_sim_function(self, func, args):
        res = func(*args)
        if type(res) != list and type(res) != tuple:
            res = (res,)
        if res[0] != sim.simx_return_ok:
            print(res[0])
            rospy.logerr('Error calling simulation')
            #raise Exception
        if len(res) == 1:
            return None
        if len(res) == 2:
            return res[1]
        return res[1:]

    def set_position_to_zero(self):
        self.run_sim_function(sim.simxSetObjectPosition, (self.clientID, self.robot, -1, [0.0, 0.0, 0.0], sim.simx_opmode_blocking))

    def add_drag_force(self, force):
        self.run_sim_function(sim.simxCallScriptFunction, (self.clientID, "Rob", sim.sim_scripttype_childscript,
                                                           "addDragForce",
                                                           [self.robot], list(force), [""], bytearray(),
                                                           sim.simx_opmode_blocking))

    def add_buoyancy_force(self, loc, force):
        self.run_sim_function(sim.simxCallScriptFunction, (self.clientID, "Rob", sim.sim_scripttype_childscript,
                                                           "addBuoyancyForce",
                                                           [self.robot], list(loc) + list(force), [""], bytearray(),
                                                           sim.simx_opmode_blocking))
    def add_thruster_force(self, loc, force):
        inp = itertools.chain.from_iterable(force)
        self.run_sim_function(sim.simxCallScriptFunction, (self.clientID, "Rob", sim.sim_scripttype_childscript,
                                                           "set_thruster_forces",
                                                           [], list(inp), [""], bytearray(),
                                                           sim.simx_opmode_blocking))

    def get_mass(self):
        out = self.run_sim_function(sim.simxCallScriptFunction, (self.clientID, "Rob", sim.sim_scripttype_childscript,
                                                                 "getMass",
                                                                 [self.robot], [], [""], bytearray(),
                                                                 sim.simx_opmode_blocking))
        return out[1][0]

    def get_pose(self, mode=sim.simx_opmode_buffer):
        pos = self.run_sim_function(sim.simxGetObjectPosition, (self.clientID, self.robot, -1, mode))
        quat = self.run_sim_function(sim.simxGetObjectQuaternion, (self.clientID, self.robot, -1, mode))
        return Pose(position=Point(*pos), orientation=Quaternion(*quat))

    def get_twist(self, mode=sim.simx_opmode_buffer):
        lin, ang = self.run_sim_function(sim.simxGetObjectVelocity, (self.clientID, self.robot, mode))
        return Twist(linear=Vector3(*lin), angular=Vector3(*ang))

    def get_gravity(self, mode=sim.simx_opmode_buffer):
        arr = self.run_sim_function(sim.simxGetArrayParameter, (self.clientID, sim.sim_arrayparam_gravity, mode))
        return Vector3(*arr)

    def get_size(self, mode=sim.simx_opmode_buffer):
        xsizemin = self.run_sim_function(sim.simxGetObjectFloatParameter, (self.clientID, self.robot, 15, mode))
        ysizemin = self.run_sim_function(sim.simxGetObjectFloatParameter, (self.clientID, self.robot, 16, mode))
        zsizemin = self.run_sim_function(sim.simxGetObjectFloatParameter, (self.clientID, self.robot, 17, mode))
        xsizemax = self.run_sim_function(sim.simxGetObjectFloatParameter, (self.clientID, self.robot, 18, mode))
        ysizemax = self.run_sim_function(sim.simxGetObjectFloatParameter, (self.clientID, self.robot, 19, mode))
        zsizemax = self.run_sim_function(sim.simxGetObjectFloatParameter, (self.clientID, self.robot, 20, mode))
        return (xsizemax - xsizemin, ysizemax - ysizemin, zsizemax - zsizemin)
