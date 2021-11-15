import sim
import rospy
import sys
from geometry_msgs.msg import Pose, Quaternion, Point, Twist, Vector3
from custom_msgs.msg import SimObject, SimObjectArray
import itertools
import re
import resource_retriever as rr
import yaml


class SimHandle:
    DOCKER_IP = '192.168.65.2'

    def __init__(self):
        sim.simxFinish(-1)
        self.clientID = sim.simxStart(self.DOCKER_IP, 5555, True, True, 5000, 5)
        if self.clientID == -1:
            rospy.logerr('Failed connecting to remote API server')
            sim.simxFinish(-1)
            sys.exit(1)
        rospy.loginfo('Connected to remote API server')
        rospy.loginfo('Testing connection')
        objs = self.run_sim_function(sim.simxGetObjects, (self.clientID, sim.sim_handle_all, sim.simx_opmode_blocking))
        rospy.loginfo(f'Number of objects in the scene: {len(objs)}')
        self.robot = self.run_sim_function(sim.simxGetObjectHandle, (self.clientID, "Rob", sim.simx_opmode_blocking))
        rospy.sleep(0.1)
        self.init_streaming()

        with open(rr.get_filename('package://simulation/data/config.yaml', use_protocol=False)) as f:
            data = yaml.safe_load(f)

        self.obj_names = '|'.join(data['cv_object'])
        self.pattern = re.compile(self.obj_names)
        self.run_sim_function(sim.simxPauseCommunication, (self.clientID, True))
        _, _, _, names = self.run_sim_function(sim.simxGetObjectGroupData,
                                                     (self.clientID, sim.sim_object_shape_type,
                                                      0, sim.simx_opmode_blocking))
        print(names)
        for name in names: ## FIXME need name in next line
            self.run_custom_sim_function(name, "reset")
            if name in [val['name'] for val in data['buoyancy']]:
                self.run_custom_sim_function(name, "enableBuoyancyDrag", ints=[1])
                self.run_custom_sim_function(name, "setDragCoefficient", ints=[1])
                self.run_custom_sim_function(name, "setDragType", ints=[1])
                self.run_custom_sim_function(name, "setMass", ints=[1]) ##FIXME: change to actual mass
        self.run_sim_function(sim.simxPauseCommunication, (self.clientID, False))

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

    def run_custom_sim_function(self, obj_name, func_name, ints=[], floats=[], strs=[""], bytes=bytearray(), mode=sim.simx_opmode_blocking):
        return self.run_sim_function(sim.simxCallScriptFunction,
                                          (self.clientID, obj_name, sim.sim_scripttype_childscript,
                                           func_name,
                                           ints, floats, strs, bytes,
                                           mode))

    def set_thruster_force(self, force):
        inp = itertools.chain.from_iterable(force)
        self.run_custom_sim_function("Rob", "setThrusterForces", floats=list(inp))

    def get_mass(self):
        return self.run_sim_function(sim.simxGetObjectIntParameter, (self.robot, sim.sim_shapefloatparam_mass, sim.simx_opmode_blocking))

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

    # Returns a SimObjArray message consisting of
    # SimObj messages representing the bounding boxes of all 
    # objects in the scene which are relevant to CV.
    def get_sim_objects(self, mode=sim.simx_opmode_blocking):
        object_array = SimObjectArray()
        _, _, _, names = self.run_sim_function(sim.simxGetObjectGroupData,(self.clientID, sim.sim_object_shape_type, 0, mode))
        filtered_names = [name for name in names if self.pattern.fullmatch(name)]
        for name in filtered_names:
            obj_handle = self.run_sim_function(sim.simxGetObjectHandle, (self.clientID, name, mode))
            instance_sim_object = SimObject()
            instance_sim_object.label = name
            instance_sim_object.points = self.get_corners(obj_handle)
            object_array.objects.append(instance_sim_object)
        return object_array
