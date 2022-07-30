#!/usr/bin/env python

from functools import total_ordering
from statistics import mean
from tkinter import image_names

from numpy import object_, array_equal
from zmq import curve_public
from task_utils import cv_object_position, object_vector, ObjectVisibleTask
import smach
import rospy
from task import Task
from move_tasks import MoveToPoseLocalTask, AllocateVelocityLocalTask, AllocateVelocityLocalForeverTask, MoveToPoseGlobalTask, MoveToMutablePoseGlobalTask
from tf import TransformListener
from time import sleep
from math import *
from custom_msgs.msg import CVObject
from geometry_msgs.msg import Pose


SIDE_THRESHOLD = 0.1  # means gate post is within 1 tenth of the side of the frame
CENTERED_THRESHOLD = 0.1  # means gate will be considered centered if within 1 tenth of the center of the frame
GATE_TICK_CV_NAME = "bin" # change back to gate_tick

"""
Plan for task restructure:

INPUT FROM CMD copper or bootlegger, direction to rotate, time to wait and moving average

1) rotate ~0.25 radians
2) check if image is in camera frame, maybe see if in center
    a) if not in frame, go to 1
    b) if in frame, proceed
3) moving average for some time to find location to move to
4) move to position ~0.5 below and ~1 past image
5) done
"""

def main():
    rospy.init_node('gate_task')
    
    #needs direction to work
    sm = create_simple_gate_task_sm()
    sleep(2)
    # Execute SMACH plan
    outcome = sm.execute()

# Rotate direction is +1 or -1 depending on how we should rotate
def create_gate_task_sm_DEFUNCT(rotate_direction):
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    listener = TransformListener()
    gate_euler_position = [0, 0, 0, 0, 0, 0]
    image_name = "bootlegger"
    with sm:
        smach.StateMachine.add('CHECK_IMAGE_VISIBLE', ObjectVisibleTask(image_name, 3),
                                transitions={
                                    'undetected': 'ROTATE_TO_GATE',
                                    'detected': 'SURVEY_GATE'
                                })
        
        smach.StateMachine.add('ROTATE_TO_GATE', MoveToPoseLocalTask(0, 0, 0, 0, 0, 0.25 * rotate_direction, listener),
                                transitions={
                                    'done': 'CHECK_IMAGE_VISIBLE'
                                })

        smach.StateMachine.add('SURVEY_GATE', SurveyGateTask(image_name, 20, gate_euler_position),
                                transitions={
                                    'done': 'MOVE_THROUGH_GATE'
                                })

        smach.StateMachine.add('MOVE_THROUGH_GATE', MoveToPoseLocalTask(*gate_euler_position, listener),
                                transitions={
                                    'done': 'succeeded'
                                })
    return sm

# SIMPLE version below


def create_simple_gate_task_sm(rotate_direction):
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    listener = TransformListener()
    global_object_position = [0, 0, 0, 0, 0, 0]
    image_name = "bootleggerbouy"
    with sm:
        smach.StateMachine.add('DIVE_TO_GATE', MoveToPoseGlobalTask(0, 0, -2, 0, 0, 0, listener),
                                transitions={
                                    'done': 'SURVEY_GATE_IMAGE_LOCATION'
                                })

        smach.StateMachine.add('SURVEY_GATE_IMAGE_LOCATION', SurveyGateImage(image_name, 1, 3),
                                transitions={
                                    'undetected': 'ROTATE_TO_GATE',
                                    'detected': 'MOVE_TO_GATE'
                                })
        
        smach.StateMachine.add('ROTATE_TO_GATE', MoveToPoseLocalTask(0, 0, 0, 0, 0, 0.25 * rotate_direction, listener),
                                transitions={
                                    'done': 'CHECK_IMAGE_VISIBLE'
                                })

        smach.StateMachine.add('MOVE_TO_GATE', MoveToPoseLocalTask(*global_object_position, listener),
                                transitions={
                                    'done': 'succeeded'
                                })

    return sm

def create_simplest_gate_task_sm():
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    listener = TransformListener()
    with sm:
        smach.StateMachine.add('MOVE_1', MoveToPoseLocalTask(2.5, 0, -2, 0, 0, 0, listener),
                                transitions={
                                    'done': 'MOVE_2'
                                })
        
        smach.StateMachine.add('MOVE_2', MoveToPoseLocalTask(2.5, 0, 0, 0, 0, 0, listener),
                                transitions={
                                    'done': 'succeeded'
                                })

    return sm

class SurveyGateImage(Task):
    def __init__(self, object_name, time, global_object_position):
        super(SurveyGateTask, self).__init__(["undetected", "detected"])
        self.object_name = object_name
        self.time = time
        self.global_object_position = global_object_position

    def run(self):
        millis = 200 # for 5 times per second
        rate = rospy.Rate(millis)
        total = 0

        x_offset = 5
        z_offset = -1

        last_cv_object_position = cv_object_position(self.cv_data[self.image_name])
        while total < self.time * 1000:
            cur_cv_object_position = cv_object_position(self.cv_data[self.image_name])
            if cur_cv_object_position is not None and not array_equal(cur_cv_object_position, last_cv_object_position):
                self.global_object_position = [ cur_cv_object_position[0] + x_offset,
                                                cur_cv_object_position[1],
                                                cur_cv_object_position[2] + z_offset,
                                                0,
                                                0,
                                                0, ]
                return "detected"
            last_cv_object_position = cur_cv_object_position
            total += millis
            rate.sleep()
        return "undetected"


class SurveyGateTask(Task):
    def __init__(self, object_name, time, output_euler_position):
        super(SurveyGateTask, self).__init__(["done"])
        self.object_name = object_name
        self.time = time
        self.output_euler_position = output_euler_position
    
    def run(self, userdata):
        millis = 10
        rate = rospy.Rate(millis)
        total = 0
        captured_vectors = []

        while total < self.time * 1000:
            gate_vector = object_vector(self.cv_data[self.object_name])
            if gate_vector is not None:
                captured_vectors.append(gate_vector)
            total += millis
            rate.sleep()
        
        avg_x = mean([v[0] for v in captured_vectors]) + 1
        avg_y = mean([v[1] for v in captured_vectors])
        avg_z = mean([v[2] for v in captured_vectors]) - 0.5
        self.output_euler_position = [avg_x, avg_y, avg_z, 0, 0, 0]
        return "done"

# class PoseFromVectorsTask(Task):
#     def __init__(self, direction_arg, *coefficients):
#         super(PoseFromVectorsTask, self).__init__(["done"],
#                                 input_keys=['vector1', 'vector2'],
#                                 output_keys=['x', 'y', 'z', 'roll', 'pitch', 'yaw'])
#         self.direction_arg = direction_arg
#         self.coefficients = coefficients

#     def run(self, userdata):
#         pos = Vector3()
#         for i in range(1, len(self.coefficients) + 1):
#             print(userdata["vector" + str(i)])
#             pos.x += userdata["vector" + str(i)].x * self.coefficients[i-1]
#             pos.y += userdata["vector" + str(i)].y * self.coefficients[i-1]
#             pos.z += userdata["vector" + str(i)].z * self.coefficients[i-1]

#         dir_vec = userdata["vector" + str(self.direction_arg)]

#         userdata.x = pos.x
#         userdata.y = pos.y
#         userdata.z = pos.z

#         userdata.roll = 0
#         userdata.pitch = pi / 2 - arccos(-dir_vec.z)
#         userdata.yaw = atan2(-dir_vec.y, -dir_vec.x)

#         print(pos.x, pos.y, pos.z)

#         return "done"



# class GateSpinDirectionTask(Task):
#     def __init__(self, threshold):
#         super(GateSpinDirectionTask, self).__init__(["center","right","left"])
#         self.threshold = threshold

#     def run(self, userdata):
#         gate_info = _scrutinize_gate(self.cv_data['gate'], self.cv_data[GATE_TICK_CV_NAME])
#         if gate_info:
#             if abs(gate_info["offset_h"]) < self.threshold:
#                 return "center"
#             if gate_info["offset_h"] < 0:
#                 return "right"
#             return "left"
#         # default to right if we can't find the gate
#         return "right"

# class GateRotationDoneTask(Task):
#     def __init__(self, threshold):
#         super(GateRotationDoneTask, self).__init__(["done"])
#         self.threshold = threshold

#     def run(self, userdata):
#         rate = rospy.Rate(15)
#         while True:
#             gate_info = _scrutinize_gate(self.cv_data['gate'], self.cv_data[GATE_TICK_CV_NAME])
#             if gate_info and abs(gate_info["offset_h"]) < self.threshold:
#                 return "done"
#             rate.sleep()

# class RollDoneTask(Task):
#     def __init__(self, roll_amount):
#         super(RollDoneTask, self).__init__(["done"])
#         self.roll_amount = roll_amount

#     def run(self, userdata):
#         startAngle = self.state.pose.pose.roll
        
#         rate = rospy.Rate(15)
#         while abs(self.state.pose.pose.roll - startAngle) < 4 * pi:
#             rate.sleep()

#         return "done"


# class GateVerticalAlignmentTask(Task):
#     def __init__(self, threshold):
#         super(GateVerticalAlignmentTask, self).__init__(["center","top","bottom","spin"])
#         self.threshold = threshold

#     def run(self, userdata):
#         gate_info = _scrutinize_gate(self.cv_data['gate'], self.cv_data[GATE_TICK_CV_NAME])
#         if gate_info:

#             if abs(gate_info["offset_v"]) < self.threshold:
#                 return "center"
#             if gate_info["offset_v"] < 0:
#                 return "top"
#             return "bottom"
#         return "spin"

    # def run(self, userdata):
    #     gate_info = _scrutinize_gate(self.cv_data['gate'], self.cv_data[GATE_TICK_CV_NAME])
    #     if gate_info:

    #         if abs(gate_info["offset_v"]) < self.threshold:
    #             return "center"
    #         if gate_info["offset_v"] < 0:
    #             return "top"
    #         return "bottom"
    #     return "spin"


# class NearGateTask(Task):
#     def __init__(self, threshold):
#         super(NearGateTask, self).__init__(["true","false","spin"])
#         self.threshold = threshold

#     def run(self, userdata):
#         gate_info = _scrutinize_gate(self.cv_data['gate'], self.cv_data[GATE_TICK_CV_NAME])
#         if gate_info:
#             if (gate_info["left"] < self.threshold) and (gate_info["right"] < self.threshold):
#                 return "true"
#             else:
#                 return "false"
#         return "spin"

# class CalcGatePoseTask(Task):
#     def __init__(self, listener):
#         super(CalcGatePoseTask, self).__init__(["done"], output_keys=['vector1','vector2'])
#         self.listener = listener

#     def run(self, userdata):
#         rate = rospy.Rate(10)
#         while self.cv_data['gateleftchild'] == None or self.cv_data['gaterightchild'] == None:
#             rate.sleep()
#         v1, v2 = _find_gate_normal_and_center(self.cv_data['gateleftchild'], self.cv_data['gaterightchild'], self.listener)
        
#         userdata['vector1'] = v1
#         userdata['vector2'] = v2

#         return "done"

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

# past_gate_info = []

# def _find_gate_normal_and_center(gate_data_l, gate_data_r, listener):
#     global past_gate_info
#     top_left = _real_pos_from_cv((gate_data_l.xmin + gate_data_l.xmax)/2, gate_data_l.ymin, gate_data_l.distance, listener)
#     top_right = _real_pos_from_cv((gate_data_r.xmin + gate_data_r.xmax)/2, gate_data_r.ymin, gate_data_r.distance, listener)
#     bottom_left = _real_pos_from_cv((gate_data_l.xmin + gate_data_l.xmax)/2, gate_data_l.ymax, gate_data_l.distance, listener)
#     bottom_right = _real_pos_from_cv((gate_data_r.xmin + gate_data_r.xmax)/2, gate_data_r.ymax, gate_data_r.distance, listener)

#     # Midpoint between top_left and bottom_right
#     center_pt = Vector3(x=(top_left.position.x + bottom_right.position.x) / 2, y=(top_left.position.y + bottom_right.position.y) / 2, z=(top_left.position.z + bottom_right.position.z) / 2)

#     diag_1 = Vector3(top_left.position.x - bottom_right.position.x, top_left.position.y - bottom_right.position.y, top_left.position.z - bottom_right.position.z)
#     diag_2 = Vector3(bottom_left.position.x - top_right.position.x, bottom_left.position.y - top_right.position.y, bottom_left.position.z - top_right.position.z)

#     # FIXME temporarily setting z to 0
#     center_pt.z = 0
#     normal = normalize(cross(diag_1,diag_2))

#     if len(past_gate_info) >= 5:
#         sum = [Vector3(), Vector3()]
#         for info in past_gate_info:
#             sum[0] = add_vectors(sum[0], info[0])
#             sum[1] = add_vectors(sum[1], info[1])
#         sum[0] = divide_vector(sum[0], len(past_gate_info))
#         sum[1] = divide_vector(sum[1], len(past_gate_info))
#         if vect_dist(normal, sum[0]) + vect_dist(center_pt, sum[1]) > 0.5:
#             return (sum[0], sum[1])
    
#     past_gate_info.append((normal, center_pt))
#     if len(past_gate_info) > 5:
#         past_gate_info.pop(0)

#     return (normal, center_pt)

# # add two vectors
# def add_vectors(v1, v2):
#     return Vector3(x=v1.x + v2.x, y=v1.y + v2.y, z=v1.z + v2.z)

# # divide a vector by a scalar
# def divide_vector(v, s):
#     return Vector3(x=v.x / s, y=v.y / s, z=v.z / s)

# # calculate distance between two vectors
# def vect_dist(v1, v2):
#     return sqrt(pow(v1.x - v2.x, 2) + pow(v1.y - v2.y, 2) + pow(v1.z - v2.z, 2))

# # calculate cross product of two vectors
# def cross(v1, v2):
#     return Vector3(x=v1.y*v2.z - v1.z*v2.y, y=v1.z*v2.x - v1.x*v2.z, z=v1.x*v2.y - v1.y*v2.x)

# # normalize vector
# def normalize(v):
#     mag = sqrt(v.x**2 + v.y**2 + v.z**2)
#     if mag == 0:
#         return Vector3(0, 0, 0)
#     return Vector3(x=v.x/mag, y=v.y/mag, z=v.z/mag)

# """Get the position of a point in global coordinates from its position from the camera
# Parameters:
# x: x position of the point relative to the left of the frame (0 to 1)
# y: y position of the point relative to the top of the frame (0 to 1)
# d: distance from the camera to the point
# """
# def _real_pos_from_cv(x, y, d, listener):
#     # TODO make sure these values are correct
#     horizontal_fov = 0.933
#     vertical_fov = 0.586
#     cam_pos_x = 0
#     cam_pos_y = 0
#     cam_pos_z = 0
#     # Fill in camera parameters later
#     # Use radians
#     cam_yaw = (x - 0.5) * horizontal_fov
#     cam_pitch = (y - 0.5) * vertical_fov

#     phi = pi/2 + cam_pitch
#     theta = -cam_yaw

#     x_cam = d * sin(phi) * cos(theta)
#     y_cam = d * sin(phi) * sin(theta)
#     z_cam = d * cos(phi)

#     # Fill in camera position relative to robot
#     local_pt = Point(x=x_cam + cam_pos_x, y=y_cam + cam_pos_y, z=z_cam + cam_pos_z)

#     local_pose = Pose(position=local_pt, orientation=Quaternion(0, 0, 0, 1))

#     return task_utils.transform_pose(listener, 'base_link', 'odom', local_pose)


# def _scrutinize_gate(gate_data, gate_tick_data):
#     """Finds the distance from the gate to each of the four edges of the frame
#     Parameters:
#     gate_data (custom_msgs/CVObject): cv data for the gate
#     gate_tick_data (custom_msgs/CVObject): cv data for the gate tick
#     Returns:
#     dict: left - distance from left edge of gate to frame edge (from 0 to 1)
#             right - distance from right edge of gate to frame edge (from 0 to 1)
#             top - distance from top edge of gate to frame edge (from 0 to 1)
#             bottom - distance from bottom edge of gate to frame edge (from 0 to 1)
#             offset_h - difference between distances on the right and left sides (from 0 to 1)
#             offset_v - difference between distances on the top and bottom sides (from 0 to 1)
#     """
#     print(gate_data)
#     if not(gate_data) or gate_data.label == 'none':
#         return None

#     res = {
#         "left": gate_data.xmin,
#         "right": 1 - gate_data.xmax,
#         "top": gate_data.ymin,
#         "bottom": 1 - gate_data.ymax
#     }

#     # Adjust the target area if the gate tick is detected
#     if gate_tick_data and gate_tick_data.label != 'none' and gate_tick_data.score > 0.5:
#         # If the tick is closer to the left side
#         if abs(gate_tick_data.xmin - gate_data.xmin) < abs(gate_tick_data.xmax - gate_data.xmax):
#             res["right"] = 1 - gate_tick_data.xmax
#         else:
#             res["left"] = gate_tick_data.xmin
        
#         res["bottom"] = 1 - gate_tick_data.ymax

#     res["offset_h"] = res["right"] - res["left"]
#     res["offset_v"] = res["bottom"] - res["top"]

#     return res

if __name__ == '__main__':
    main()