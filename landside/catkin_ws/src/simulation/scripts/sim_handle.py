import sim
import rospy
import sys
from geometry_msgs.msg import Pose, Quaternion, Point, Twist, Vector3
from custom_msgs.msg import SimObject, SimObjectArray
import itertools
import re
import resource_retriever as rr
import yaml
import traceback
import numpy as np

class SimHandle:
    DOCKER_IP = '192.168.65.2'

    def __init__(self):
        sim.simxFinish(-1)
        self.clientID = sim.simxStart(self.DOCKER_IP, 5555, True, True, 5000, 5)
        if self.clientID == -1:
            rospy.logerr('sim_handle.__init__: Failed connecting to remote API server.')
            rospy.logerr('sim_handle.__init__: Make sure the simulation is playing.')
            sim.simxFinish(-1)
            sys.exit(1)
        rospy.loginfo('sim_handle.__init__: Connected to remote API server')
        rospy.loginfo('sim_handle.__init__: Testing connection')
        objs = self.run_sim_function(sim.simxGetObjects, (self.clientID, sim.sim_handle_all, sim.simx_opmode_blocking))
        rospy.loginfo(f'sim_handle.__init__: Number of objects in the scene: {len(objs)}')
        self.robot = self.run_sim_function(sim.simxGetObjectHandle, (self.clientID, "Rob", sim.simx_opmode_blocking))
        rospy.sleep(0.1)
        self.init_streaming()

        config_filepath = rr.get_filename(
            'package://simulation/data/config.yaml', 
            use_protocol=False
        )
        with open(config_filepath) as f:
            data = yaml.safe_load(f)

        self.obj_names = f"({'|'.join(data['cv_objects'])})(#\d+)?"
        self.pattern = re.compile(self.obj_names)
        _, _, _, names = self.run_sim_function(sim.simxGetObjectGroupData,
                                                     (self.clientID, sim.sim_object_shape_type,
                                                      0, sim.simx_opmode_blocking))
        filtered_names = [name for name in names if self.pattern.fullmatch(name)]
        print(f"sim_handle.__init__: Names of relevant simulation objects: {filtered_names}")
        # self.run_sim_function(sim.simxPauseCommunication, (self.clientID, True))
        # for name in filtered_names: ## FIXME need name in next line
        #     print(f"About to run function <reset> on {name}...")
        #     self.run_custom_sim_function(name, "reset")
        #     if name in [val['name'] for val in data['buoyancy']]:
        #         print(f"About to run function enableBuoyancyDrag on {name}...")
        #         self.run_custom_sim_function(name, "enableBuoyancyDrag", ints=[1])
        #         self.run_custom_sim_function(name, "setDragCoefficient", ints=[1])
        #         self.run_custom_sim_function(name, "setDragType", ints=[1])
        #         self.run_custom_sim_function(name, "setMass", ints=[1]) ##FIXME: change to actual mass
        # self.run_sim_function(sim.simxPauseCommunication, (self.clientID, False))

        rospy.loginfo("sim_handle.__init__: Starting main loop")

    def init_sim_objects(self):
        # self.run_sim_function(sim.simxPauseCommunication, (self.clientID, True))
        for name in filtered_names: ## FIXME need name in next line
            print(f"About to run function <reset> on {name}...")
            self.run_custom_sim_function(name, "reset")
            if name in [val['name'] for val in data['buoyancy']]:
                # The opmode should not be blocking, since we don't expect
                # to get a response from the simulation until we turn communications
                # back on.
                paused_comms_opmode = simx_opmode_oneshot
                paused_comms_opmode = simx_opmode_blocking
                self.run_custom_sim_function(name, "enableBuoyancyDrag", ints=[1], mode=paused_comms_opmode)
                self.run_custom_sim_function(name, "setDragCoefficient", ints=[1], mode=paused_comms_opmode)
                self.run_custom_sim_function(name, "setDragType", ints=[1], mode=paused_comms_opmode)
                self.run_custom_sim_function(name, "setMass", ints=[1], mode=paused_comms_opmode) ##FIXME: change to actual mass
        # self.run_sim_function(sim.simxPauseCommunication, (self.clientID, False))

    def init_streaming(self):
        self.get_pose(mode=sim.simx_opmode_streaming)
        self.get_twist(mode=sim.simx_opmode_streaming)

    def run_sim_function(self, func, args):
        res = func(*args)
        if not isinstance(res, list) and not isinstance(res, tuple):
            res = (res,)
        if res[0] != sim.simx_return_ok and args[-1] != sim.simx_opmode_streaming:
            rospy.logerr(f'sim_handle.run_sim_function: Error calling simulation. Code: {res[0]}')
            rospy.logerr(f'sim_handle.run_sim_function: Function: {func}')
            rospy.logerr(f'sim_handle.run_sim_function: Args: {args}')
            traceback.print_stack()

            if res[0] == 3: # FIXME: Replace this with sim.simx_??? const
                rospy.logerr(f'Command timed out. If running a custom ' + \
                'function, make sure the function exists on the target object.')
            elif res[0] == sim.simx_return_remote_error_flag:
                rospy.logerr(f'Return remote error flag. This occurs when ' + \
                'the client uses the wrong opmode.')
                raise Exception('Failed to run sim function!')
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
        mass = self.run_sim_function(sim.simxGetObjectFloatParameter, (self.clientID, self.robot, sim.sim_shapefloatparam_mass, sim.simx_opmode_blocking))
        print(f"sim_handle.get_mass: robot mass: {mass}")
        return mass

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
        robot_handle = self.run_sim_function(sim.simxGetObjectHandle, (self.clientID, "Rob", sim.simx_opmode_blocking))
        robot_x, robot_y, robot_z = self.run_sim_function(sim.simxGetObjectPosition,
                                            (self.clientID, robot_handle, -1, mode))
        for name in filtered_names:
            obj_handle = self.run_sim_function(sim.simxGetObjectHandle, (self.clientID, name, mode))
            instance_sim_object = SimObject()
            instance_sim_object.label = name
            instance_sim_object.distance = self.get_distance(obj_handle, robot_x, robot_y, robot_z)
            instance_sim_object.points = self.get_corners(obj_handle)
            object_array.objects.append(instance_sim_object)
        return object_array

    def get_distance(self, obj_handle, robot_x, robot_y, robot_z, mode=sim.simx_opmode_blocking):
        """
        Given the robot's position and the handle of a simulation object,
        returns the distance from the robot's center to the object's
        center. This is probably the center of the minimal bounding box
        around the object.

        DISCLAIMER: You will need to adjust for the fact that this isn't
        the distance from the object to the CAMERA.
        """
        base_x, base_y, base_z = self.run_sim_function(sim.simxGetObjectPosition,
                                                       (self.clientID, obj_handle, -1, mode))
        distance_x = robot_x - base_x
        distance_y = robot_y - base_y
        distance_z = robot_z - base_z
        return np.sqrt(distance_x**2 + distance_y**2 + distance_z**2)