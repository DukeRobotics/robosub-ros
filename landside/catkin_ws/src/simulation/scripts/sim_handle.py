import sim
import rospy
import sys
from geometry_msgs.msg import Pose, Quaternion, Point, Twist, Vector3
from custom_msgs.msg import SimObject, SimObjectArray
import itertools


class SimHandle:
    DOCKER_IP = '192.168.65.2'
    OBJ_NAMES = ['Gate', 'BootleggerBuoy']

    def __init__(self):
        sim.simxFinish(-1)
        self.clientID = sim.simxStart(self.DOCKER_IP, 8080, True, True, 5000, 5)
        if self.clientID == -1:
            rospy.logerr('Failed connecting to remote API server')
            sim.simxFinish(-1)
            sys.exit(1)
        rospy.loginfo('Connected to remote API server')
        rospy.loginfo('Testing connection')
        objs = self.run_sim_function(sim.simxGetObjects, (self.clientID, sim.sim_handle_all, sim.simx_opmode_blocking))
        rospy.loginfo(f'Number of objects in the scene: {len(objs)}')
        self.robot = self.run_sim_function(sim.simxGetObjectHandle, (self.clientID, "Rob", sim.simx_opmode_blocking))
        gate_names = ["Gate", "GateLeftChild", "GateRightChild"]
        self.gate = [self.run_sim_function(sim.simxGetObjectHandle,
                                           (self.clientID, name, sim.simx_opmode_blocking)) for name in gate_names]
        self.set_position_to_zero()
        rospy.sleep(0.1)
        self.init_streaming()
        rospy.loginfo("Starting main loop")

    def init_streaming(self):
        self.get_pose(mode=sim.simx_opmode_streaming)
        self.get_twist(mode=sim.simx_opmode_streaming)

    def run_sim_function(self, func, args):
        res = func(*args)
        if not isinstance(res, list) and not isinstance(res, tuple):
            res = (res,)
        if res[0] != sim.simx_return_ok and args[-1] != sim.simx_opmode_streaming:
            rospy.logerr(f'Error calling simulation. Code: {res[0]}')
        if len(res) == 1:
            return None
        if len(res) == 2:
            return res[1]
        return res[1:]

    def set_position_to_zero(self):
        self.run_sim_function(sim.simxSetObjectPosition, (self.clientID, self.robot,
                                                          -1, [0.0, 0.0, 0.0], sim.simx_opmode_blocking))

    def set_thruster_force(self, force):
        inp = itertools.chain.from_iterable(force)
        self.run_sim_function(sim.simxCallScriptFunction, (self.clientID, "Rob", sim.sim_scripttype_childscript,
                                                           "setThrusterForces",
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

    def get_corners(self, obj, mode=sim.simx_opmode_blocking):
        ret = []
        base_x, base_y, base_z = self.run_sim_function(sim.simxGetObjectPosition,
                                                       (self.clientID, obj, -1, mode))

        min_x = self.run_sim_function(sim.simxGetObjectFloatParameter, (self.clientID, obj, 15, mode))
        min_y = self.run_sim_function(sim.simxGetObjectFloatParameter, (self.clientID, obj, 16, mode))
        min_z = self.run_sim_function(sim.simxGetObjectFloatParameter, (self.clientID, obj, 17, mode))

        max_x = self.run_sim_function(sim.simxGetObjectFloatParameter, (self.clientID, obj, 18, mode))
        max_y = self.run_sim_function(sim.simxGetObjectFloatParameter, (self.clientID, obj, 19, mode))
        max_z = self.run_sim_function(sim.simxGetObjectFloatParameter, (self.clientID, obj, 20, mode))
        for i in range(2):
            for j in range(2):
                for k in range(2):
                    point_x = base_x + (min_x if i == 0 else max_x)
                    point_y = base_y + (min_y if j == 0 else max_y)
                    point_z = base_z + (min_z if k == 0 else max_z)
                    ret.append(Point(x=point_x, y=point_y, z=point_z))
        return ret

    def get_gate_corners(self, mode=sim.simx_opmode_blocking):
        gate_sim_object = SimObject()
        gate_sim_object.label = 'gate'

        for gate_obj in self.gate:
            gate_sim_object.points += self.get_corners(gate_obj)
        return gate_sim_object

    # Returns a SimObjArray message consisting of
    # SimObj messages representing the bounding boxes of all 
    # objects in the scene which are relevant to CV.
    def get_sim_objects(self, mode=sim.simx_opmode_blocking):
        object_array = SimObjectArray()

        for obj_name in self.OBJ_NAMES:
            obj_index = -1
            while True: # gets all objects of type obj_name in simulation
                instance_name = '{obj_name}{obj_index}' if obj_index >= 0 else obj_name
                obj_index += 1
                instance_handle = self.run_sim_function(sim.simxGetObjectHandle, (self.clientID, instance_name, mode))
                print(obj_name, instance_handle)
                if instance_handle == 0:
                    break
                instance_sim_object = SimObject()
                instance_sim_object.label = obj_name.lower()

                child_index = 0
                instance_sim_object.points += self.get_corners(instance_handle)
                while True:
                    # TODO: Recursively get children so that we can have
                    # trees of objects bundled together
                    child_handle = self.run_sim_function(sim.simxGetObjectChild,
                        (self.clientID, instance_handle, child_index, mode))
                    instance_sim_object.points += self.get_corners(child_handle)
                    if child_handle == -1:
                        break
                    child_index += 1
                object_array.objects.append(instance_sim_object)
        return object_array
