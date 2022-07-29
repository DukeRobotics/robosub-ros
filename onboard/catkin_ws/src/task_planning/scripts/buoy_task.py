#!/usr/bin/env python

from functools import total_ordering
from statistics import mean
from tkinter import image_names
from task_utils import object_vector, ObjectVisibleTask
import smach
import rospy
from task import Task
from move_tasks import MoveToPoseLocalTask, AllocateVelocityLocalTask, AllocateVelocityLocalForeverTask, MoveToPoseGlobalTask, MoveToMutablePoseGlobalTask
from tf import TransformListener
from time import sleep
from math import *


SIDE_THRESHOLD = 0.1  # means buoy post is within 1 tenth of the side of the frame
CENTERED_THRESHOLD = 0.1  # means buoy will be considered centered if within 1 tenth of the center of the frame
buoy_TICK_CV_NAME = "bin" # change back to buoy_tick

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
    rospy.init_node('buoy_task')
    
    sm = create_buoy_task_sm()
    sleep(2)
    # Execute SMACH plan
    outcome = sm.execute()

# Rotate direction is +1 or -1 depending on how we should rotate
def create_buoy_task_sm(rotate_direction):
    sm = smach.StateMachine(outcomes=['buoy_task_succeeded', 'buoy_task_failed'])
    listener = TransformListener()
    buoy_euler_position = [0, 0, 0, 0, 0, 0]
    image_name = "gun"
    with sm:
        smach.StateMachine.add('CHECK_IMAGE_VISIBLE', ObjectVisibleTask(image_name, 3),
                                transitions={
                                    'undetected': 'ROTATE_TO_buoy',
                                    'detected': 'SURVEY_BUOY_1'
                                })
        
        smach.StateMachine.add('ROTATE_TO_buoy', MoveToPoseLocalTask(0, 0, 0, 0, 0, 0.25 * rotate_direction, listener),
                                transitions={
                                    'done': 'CHECK_IMAGE_VISIBLE'
                                })

        smach.StateMachine.add('SURVEY_BUOY_1', SurveyBuoyTask(image_name, 20, 0.5, buoy_euler_position),
                                transitions={
                                    'done': 'MOVE_TO_BUOY_1'
                                })

        smach.StateMachine.add('MOVE_TO_BUOY_1', MoveToPoseLocalTask(*buoy_euler_position, listener),
                                transitions={
                                    'done': 'SURVEY_BUOY_2'
                                })

        smach.StateMachine.add('SURVEY_BUOY_2', SurveyBuoyTask(image_name, 20, 0.5, buoy_euler_position),
                                transitions={
                                    'done': 'MOVE_TO_BUOY_2'
                                })

        smach.StateMachine.add('MOVE_TO_BUOY_2', MoveToPoseLocalTask(*buoy_euler_position, listener),
                                transitions={
                                    'done': 'SURVEY_BUOY_3'
                                })

        smach.StateMachine.add('SURVEY_BUOY_3', SurveyBuoyTask(image_name, 20, 1, buoy_euler_position),
                                transitions={
                                    'done': 'MOVE_TO_BUOY_3'
                                })

        smach.StateMachine.add('MOVE_TO_BUOY_3', MoveToPoseLocalTask(*buoy_euler_position, listener),
                                transitions={
                                    'done': 'MOVE_AWAY_FROM_BUOY'
                                })
        
        smach.StateMachine.add('MOVE_AWAY_FROM_BUOY', MoveToPoseLocalTask(0, 5, 0, 0, 0, 0, listener),
                                transitions={
                                    'done': 'buoy_task_succeeded'
                                })

    return sm


class SurveyBuoyTask(Task):
    def __init__(self, object_name, time, ratio, output_euler_position):
        super(SurveyBuoyTask, self).__init__(["done"])
        self.object_name = object_name
        self.time = time
        self.ratio = ratio
        self.output_euler_position = output_euler_position
    
    def run(self, userdata):
        millis = 10
        rate = rospy.Rate(millis)
        total = 0
        captured_vectors = []

        while total < self.time * 1000:
            buoy_vector = object_vector(self.cv_data[self.object_name])
            if buoy_vector is not None:
                captured_vectors.append(buoy_vector)
            total += millis
            rate.sleep()
        
        avg_x = mean([v[0] for v in captured_vectors]) * self.ratio
        avg_y = mean([v[1] for v in captured_vectors]) * self.ratio
        avg_z = mean([v[2] for v in captured_vectors]) * self.ratio
        self.output_euler_position = [avg_x, avg_y, avg_z, 0, 0, 0]
        return "done"


if __name__ == '__main__':
    main()