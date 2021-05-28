import sim
import rospy
import sys
import traceback
from geometry_msgs.msg import Pose, Quaternion, Point, Twist, Vector3
import numpy as np

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
        rospy.loginfo("Starting main loop")

    def run_sim_function(self, func, args):
        res = func(*args)
        if type(res) != list and type(res) != tuple:
            res = (res,)
        if res[0] != sim.simx_return_ok:
            print(res[0])
            rospy.logerr('Error calling simulation')
            rospy.logerr(traceback.format_exc())
            sys.exit(1)
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
        self.run_sim_function(sim.simxCallScriptFunction, (self.clientID, "Rob", sim.sim_scripttype_childscript,
                                                           "addThrusterForce",
                                                           [self.robot], list(loc) + list(force), [""], bytearray(),
                                                           sim.simx_opmode_blocking))

    def get_mass(self):
        out = self.run_sim_function(sim.simxCallScriptFunction, (self.clientID, "Rob", sim.sim_scripttype_childscript,
                                                                 "getMass",
                                                                 [self.robot], [], [""], bytearray(),
                                                                 sim.simx_opmode_blocking))
        return out[1][0]

    def get_pose(self):
        pos = self.run_sim_function(
            sim.simxGetObjectPosition, (self.clientID, self.robot, -1, sim.simx_opmode_blocking))
        quat = self.run_sim_function(
            sim.simxGetObjectQuaternion, (self.clientID, self.robot, -1, sim.simx_opmode_blocking))
        return Pose(position=Point(*pos), orientation=Quaternion(*quat))

    def get_twist(self):
        lin, ang = self.run_sim_function(
            sim.simxGetObjectVelocity, (self.clientID, self.robot, sim.simx_opmode_blocking))
        return Twist(linear=Vector3(*lin), angular=Vector3(*ang))

    def get_gravity(self):
        arr = self.run_sim_function(sim.simxGetArrayParameter, (
            self.clientID, sim.sim_arrayparam_gravity, sim.simx_opmode_blocking))
        return Vector3(*arr)

    def get_size(self):
        xsizemin = self.run_sim_function(sim.simxGetObjectFloatParameter, (self.clientID, self.robot, 15, sim.simx_opmode_blocking))
        ysizemin = self.run_sim_function(sim.simxGetObjectFloatParameter, (self.clientID, self.robot, 16, sim.simx_opmode_blocking))
        zsizemin = self.run_sim_function(sim.simxGetObjectFloatParameter, (self.clientID, self.robot, 17, sim.simx_opmode_blocking))
        xsizemax = self.run_sim_function(sim.simxGetObjectFloatParameter, (self.clientID, self.robot, 18, sim.simx_opmode_blocking))
        ysizemax = self.run_sim_function(sim.simxGetObjectFloatParameter, (self.clientID, self.robot, 19, sim.simx_opmode_blocking))
        zsizemax = self.run_sim_function(sim.simxGetObjectFloatParameter, (self.clientID, self.robot, 20, sim.simx_opmode_blocking))
        return (xsizemax - xsizemin, ysizemax - ysizemin, zsizemax - zsizemin)

    def set_barrier(self):
        self.run_sim_function(sim.simxCallScriptFunction, (self.clientID, "Rob", sim.sim_scripttype_childscript,
                                                           "setBarrier",
                                                           [], [], [""], bytearray(),
                                                           sim.simx_opmode_blocking))
    def get_barrier(self):
        out = self.run_sim_function(sim.simxCallScriptFunction, (self.clientID, "Rob", sim.sim_scripttype_childscript,
                                                                 "getBarrier",
                                                                 [], [], [""], bytearray(),
                                                                 sim.simx_opmode_blocking))
        return out[0][0]    
