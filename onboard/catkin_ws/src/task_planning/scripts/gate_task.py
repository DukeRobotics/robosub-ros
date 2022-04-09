#!/usr/bin/env python

from urllib.parse import uses_relative
from numpy import arccos
from task_utils import parse_pose
import smach
import rospy
import task_utils
from task import Task
from move_tasks import MoveToPoseLocalTask, AllocateVelocityLocalTask, AllocateVelocityLocalForeverTask, MoveToPoseGlobalTask, MoveToMutablePoseGlobalTask
from tf import TransformListener
from time import sleep
from geometry_msgs.msg import Pose, Quaternion, Twist, Point, Vector3
from math import *


SIDE_THRESHOLD = 0.1  # means gate post is within 1 tenth of the side of the frame
CENTERED_THRESHOLD = 0.1  # means gate will be considered centered if within 1 tenth of the center of the frame
GATE_TICK_CV_NAME = "bin" # change back to gate_tick

def main():
    rospy.init_node('gate_task')
    
    sm = create_gate_task_sm()
    sleep(2)
    # Execute SMACH plan
    outcome = sm.execute()

def create_gate_task_sm(velocity=0.2):
    sm = smach.StateMachine(outcomes=['gate_task_succeeded', 'gate_task_failed'])
    listener = TransformListener()
    STANDARD_MOVE_SPEED = 3
    METERS_FROM_GATE = 2
    MOVE_THROUGH_GATE_SPEED = 0.1
    with sm:
        # TODO add "dive and move away from dock task"
        # smach.StateMachine.add('NEAR_GATE', NearGateTask(SIDE_THRESHOLD),
        #                        transitions={
        #                            'true': 'MOVE_TO_GATE_FRONT',
        #                            'false': 'CHOOSE_ROTATE_DIR',
        #                            'spin': 'NEAR_GATE'})

        smach.StateMachine.add('CHOOSE_ROTATE_DIR', GateSpinDirectionTask(CENTERED_THRESHOLD),
                               transitions={
                                   'left': 'ROTATE_TO_GATE_LEFT',
                                   'right': 'ROTATE_TO_GATE_RIGHT',
                                   'center': 'MOVE_TO_GATE_FRONT'})

        def concurrence_term_rotate_loop_cb(outcome_map):
            return outcome_map['ROTATION_DONE'] == 'done'

        rotate_gate_left_cc = smach.Concurrence(outcomes = ['done'],
                 default_outcome = 'done',
                 child_termination_cb = concurrence_term_rotate_loop_cb,
                 outcome_map = {'done':{'ROTATION_DONE':'done'}})
        with rotate_gate_left_cc:
            smach.Concurrence.add('ROTATION_DONE', GateRotationDoneTask(CENTERED_THRESHOLD))
            smach.Concurrence.add('ROTATE', AllocateVelocityLocalForeverTask(0, 0, 0, 0, 0, velocity))

        rotate_gate_right_cc = smach.Concurrence(outcomes = ['done'],
                 default_outcome = 'done',
                 child_termination_cb = concurrence_term_rotate_loop_cb,
                 outcome_map = {'done':{'ROTATION_DONE':'done'}})
        with rotate_gate_right_cc:
            smach.Concurrence.add('ROTATION_DONE', GateRotationDoneTask(CENTERED_THRESHOLD))
            smach.Concurrence.add('ROTATE', AllocateVelocityLocalForeverTask(0, 0, 0, 0, 0, -velocity))

        smach.StateMachine.add('ROTATE_TO_GATE_LEFT', rotate_gate_left_cc,
                                transitions={
                                   'done': 'MOVE_TO_GATE_FRONT'})

        smach.StateMachine.add('ROTATE_TO_GATE_RIGHT', rotate_gate_right_cc,
                                transitions={
                                   'done': 'MOVE_TO_GATE_FRONT'})

        # Move to 2 meters in front of the gate


        def concurrence_term_calc_loop_cb(outcome_map):
            return outcome_map['MOVE_IN_FRONT_OF_GATE'] == 'done'

        gate_calc_sm = smach.StateMachine(outcomes=['done'])
        gate_start_mutable_pose = task_utils.MutablePose()
        with gate_calc_sm:
            smach.StateMachine.add('CALC_GATE_POSE', CalcGatePoseTask(listener), 
                                transitions={
                                    'done': 'CALC_DESIRED_ROBOT_GATE_POSE'
                                })

            smach.StateMachine.add('CALC_DESIRED_ROBOT_GATE_POSE', PoseFromVectorsTask(1, METERS_FROM_GATE, 1),
                                transitions={
                                    'done': 'SET_MUTABLE_POSE'
                                })

            smach.StateMachine.add('SET_MUTABLE_POSE', task_utils.MutatePoseTask(gate_start_mutable_pose),
                                transitions={
                                    'done': 'CALC_GATE_POSE'
                                })


        move_to_gate_cc = smach.Concurrence(outcomes = ['done'],
                 default_outcome = 'done',
                 child_termination_cb = concurrence_term_calc_loop_cb,
                 outcome_map = {'done':{'MOVE_IN_FRONT_OF_GATE':'done'}})
        with move_to_gate_cc:
            smach.Concurrence.add('MOVE_IN_FRONT_OF_GATE', MoveToMutablePoseGlobalTask(gate_start_mutable_pose))
            smach.Concurrence.add('CALC_FRONT_OF_GATE', gate_calc_sm)
            

        smach.StateMachine.add('MOVE_TO_GATE_FRONT', move_to_gate_cc,
                                transitions={
                                    'done':'SPIN_THROUGH_GATE'
                                })


        spin_through_gate_cc = smach.Concurrence(outcomes = ['done'],
                 default_outcome = 'done',
                 child_termination_cb = concurrence_term_rotate_loop_cb,
                 outcome_map = {'done':{'ROTATION_DONE':'done'}})
        with spin_through_gate_cc:
            smach.Concurrence.add('ROTATION_DONE', RollDoneTask(4*pi))
            # Spin and move through the gate
            smach.Concurrence.add('MOVE_AND_ROTATE', AllocateVelocityLocalForeverTask(MOVE_THROUGH_GATE_SPEED * METERS_FROM_GATE, 0, -.001, velocity, 0, 0))


        smach.StateMachine.add('SPIN_THROUGH_GATE', spin_through_gate_cc,
                                transitions={
                                    'done':'gate_task_succeeded'
                                })



    return sm

class PoseFromVectorsTask(Task):
    def __init__(self, direction_arg, *coefficients):
        super(PoseFromVectorsTask, self).__init__(["done"],
                                input_keys=['vector1', 'vector2'],
                                output_keys=['x', 'y', 'z', 'roll', 'pitch', 'yaw'])
        self.direction_arg = direction_arg
        self.coefficients = coefficients

    def run(self, userdata):
        pos = Vector3()
        for i in range(1, len(self.coefficients) + 1):
            print(userdata["vector" + str(i)])
            pos.x += userdata["vector" + str(i)].x * self.coefficients[i-1]
            pos.y += userdata["vector" + str(i)].y * self.coefficients[i-1]
            pos.z += userdata["vector" + str(i)].z * self.coefficients[i-1]

        dir_vec = userdata["vector" + str(self.direction_arg)]

        userdata.x = pos.x
        userdata.y = pos.y
        userdata.z = pos.z

        userdata.roll = 0
        userdata.pitch = pi / 2 - arccos(-dir_vec.z)
        userdata.yaw = atan2(-dir_vec.y, -dir_vec.x)

        print(pos.x, pos.y, pos.z)

        return "done"

class GateSpinDirectionTask(Task):
    def __init__(self, threshold):
        super(GateSpinDirectionTask, self).__init__(["center","right","left"])
        self.threshold = threshold

    def run(self, userdata):
        gate_info = _scrutinize_gate(self.cv_data['gate'], self.cv_data[GATE_TICK_CV_NAME])
        if gate_info:
            if abs(gate_info["offset_h"]) < self.threshold:
                return "center"
            if gate_info["offset_h"] < 0:
                return "right"
            return "left"
        # default to right if we can't find the gate
        return "right"

class GateRotationDoneTask(Task):
    def __init__(self, threshold):
        super(GateRotationDoneTask, self).__init__(["done"])
        self.threshold = threshold

    def run(self, userdata):
        rate = rospy.Rate(15)
        while True:
            gate_info = _scrutinize_gate(self.cv_data['gate'], self.cv_data[GATE_TICK_CV_NAME])
            if gate_info and abs(gate_info["offset_h"]) < self.threshold:
                return "done"
            rate.sleep()

class RollDoneTask(Task):
    def __init__(self, roll_amount):
        super(RollDoneTask, self).__init__(["done"])
        self.roll_amount = roll_amount

    def run(self, userdata):
        startAngle = self.state.pose.pose.roll
        
        rate = rospy.Rate(15)
        while abs(self.state.pose.pose.roll - startAngle) < 4 * pi:
            rate.sleep()

        return "done"


class GateVerticalAlignmentTask(Task):
    def __init__(self, threshold):
        super(GateVerticalAlignmentTask, self).__init__(["center","top","bottom","spin"])
        self.threshold = threshold

    def run(self, userdata):
        gate_info = _scrutinize_gate(self.cv_data['gate'], self.cv_data[GATE_TICK_CV_NAME])
        if gate_info:

            if abs(gate_info["offset_v"]) < self.threshold:
                return "center"
            if gate_info["offset_v"] < 0:
                return "top"
            return "bottom"
        return "spin"


class NearGateTask(Task):
    def __init__(self, threshold):
        super(NearGateTask, self).__init__(["true","false","spin"])
        self.threshold = threshold

    def run(self, userdata):
        gate_info = _scrutinize_gate(self.cv_data['gate'], self.cv_data[GATE_TICK_CV_NAME])
        if gate_info:
            if (gate_info["left"] < self.threshold) and (gate_info["right"] < self.threshold):
                return "true"
            else:
                return "false"
        return "spin"

class CalcGatePoseTask(Task):
    def __init__(self, listener):
        super(CalcGatePoseTask, self).__init__(["done"], output_keys=['vector1','vector2'])
        self.listener = listener

    def run(self, userdata):
        v1, v2 = _find_gate_normal_and_center(self.cv_data['gateleftchild'], self.cv_data['gaterightchild'], self.listener)
        
        userdata['vector1'] = v1
        userdata['vector2'] = v2

        return "done"

"""
Algorithm plan for finding gate pos from cv data:

Get angle of gate point from center of camera using cam_yaw = (x_from_center / width (which is just 1)) * horizontal_fov
For left this would be cam_yaw = (left - 0.5) * horizontal_fov
cam_pitch = (y_from_center / height (once again 1)) * vert_fov
We now have global spherical coordinates for our point
phi = 90 + cam_pitch
theta = -cam_yaw
This has robot oriented facing +x as is usual
Then add result to camera offset
Then convert robot local coord result to global coords

Use our 4 corners to get 2 vectors, then cross product to get normal
Then position robot along that normal and whatever distance we want
"""

def _find_gate_normal_and_center(gate_data_l, gate_data_r, listener):
    top_left = _real_pos_from_cv((gate_data_l.xmin + gate_data_l.xmax)/2, gate_data_l.ymin, gate_data_l.distance, listener)
    top_right = _real_pos_from_cv((gate_data_r.xmin + gate_data_r.xmax)/2, gate_data_r.ymin, gate_data_r.distance, listener)
    bottom_left = _real_pos_from_cv((gate_data_l.xmin + gate_data_l.xmax)/2, gate_data_l.ymax, gate_data_l.distance, listener)
    bottom_right = _real_pos_from_cv((gate_data_r.xmin + gate_data_r.xmax)/2, gate_data_r.ymax, gate_data_r.distance, listener)

    # Midpoint between top_left and bottom_right
    center_pt = Vector3(x=(top_left.position.x + bottom_right.position.x) / 2, y=(top_left.position.y + bottom_right.position.y) / 2, z=(top_left.position.z + bottom_right.position.z) / 2)

    diag_1 = Vector3(top_left.position.x - bottom_right.position.x, top_left.position.y - bottom_right.position.y, top_left.position.z - bottom_right.position.z)
    diag_2 = Vector3(bottom_left.position.x - top_right.position.x, bottom_left.position.y - top_right.position.y, bottom_left.position.z - top_right.position.z)

    return (normalize(cross(diag_1,diag_2)), center_pt)

# calculate cross product of two vectors
def cross(v1, v2):
    return Vector3(x=v1.y*v2.z - v1.z*v2.y, y=v1.z*v2.x - v1.x*v2.z, z=v1.x*v2.y - v1.y*v2.x)

# normalize vector
def normalize(v):
    mag = sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
    return Vector3(x=v.x/mag, y=v.y/mag, z=v.z/mag)

"""Get the position of a point in global coordinates from its position from the camera
Parameters:
x: x position of the point relative to the left of the frame (0 to 1)
y: y position of the point relative to the top of the frame (0 to 1)
d: distance from the camera to the point
"""
def _real_pos_from_cv(x, y, d, listener):
    # TODO make sure these values are correct
    horizontal_fov = 0.933
    vertical_fov = 0.586
    cam_pos_x = 0
    cam_pos_y = 0
    cam_pos_z = 0
    # Fill in camera parameters later
    # Use radians
    cam_yaw = (x - 0.5) * horizontal_fov
    cam_pitch = (y - 0.5) * vertical_fov

    phi = pi/2 + cam_pitch
    theta = -cam_yaw

    x_cam = d * sin(phi) * cos(theta)
    y_cam = d * sin(phi) * sin(theta)
    z_cam = d * cos(phi)

    # Fill in camera position relative to robot
    local_pt = Point(x=x_cam + cam_pos_x, y=y_cam + cam_pos_y, z=z_cam + cam_pos_z)

    local_pose = Pose(position=local_pt, orientation=Quaternion(0, 0, 0, 1))

    return task_utils.transform_pose(listener, 'base_link', 'odom', local_pose)


def _scrutinize_gate(gate_data, gate_tick_data):
    """Finds the distance from the gate to each of the four edges of the frame
    Parameters:
    gate_data (custom_msgs/CVObject): cv data for the gate
    gate_tick_data (custom_msgs/CVObject): cv data for the gate tick
    Returns:
    dict: left - distance from left edge of gate to frame edge (from 0 to 1)
            right - distance from right edge of gate to frame edge (from 0 to 1)
            top - distance from top edge of gate to frame edge (from 0 to 1)
            bottom - distance from bottom edge of gate to frame edge (from 0 to 1)
            offset_h - difference between distances on the right and left sides (from 0 to 1)
            offset_v - difference between distances on the top and bottom sides (from 0 to 1)
    """
    if not(gate_data) or gate_data.label == 'none':
        return None

    res = {
        "left": gate_data.xmin,
        "right": 1 - gate_data.xmax,
        "top": gate_data.ymin,
        "bottom": 1 - gate_data.ymax
    }

    # Adjust the target area if the gate tick is detected
    if gate_tick_data and gate_tick_data.label != 'none' and gate_tick_data.score > 0.5:
        # If the tick is closer to the left side
        if abs(gate_tick_data.xmin - gate_data.xmin) < abs(gate_tick_data.xmax - gate_data.xmax):
            res["right"] = 1 - gate_tick_data.xmax
        else:
            res["left"] = gate_tick_data.xmin
        
        res["bottom"] = 1 - gate_tick_data.ymax

    res["offset_h"] = res["right"] - res["left"]
    res["offset_v"] = res["bottom"] - res["top"]

    return res

if __name__ == '__main__':
    main()