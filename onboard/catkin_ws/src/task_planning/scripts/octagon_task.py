#!/usr/bin/env python


from statistics import mean

from task_utils import object_vector, ObjectVisibleTask
import smach
import rospy
from task import Task
from move_tasks import MoveToPoseLocalTask, AllocateVelocityLocalTask, AllocateVelocityLocalForeverTask, MoveToPoseGlobalTask, MoveToMutablePoseGlobalTask
from tf import TransformListener
from time import sleep
from math import *


SIDE_THRESHOLD = 0.1  # means octagon post is within 1 tenth of the side of the frame
CENTERED_THRESHOLD = 0.1  # means octagon will be considered centered if within 1 tenth of the center of the frame
octagon_TICK_CV_NAME = "bin"  # change back to octagon_tick

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
    rospy.init_node('octagon_task')

    sm = create_eyeball_octagon_task_sm()
    sleep(2)
    # Execute SMACH plan
    outcome = sm.execute()

# Rotate direction is +1 or -1 depending on how we should rotate


def create_acoustics_task_sm(rotate_direction):
    # NOTE: THIS IS ENTIRELY WRONG
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    listener = TransformListener()
    octagon_euler_position = [0, 0, 0, 0, 0, 0]
    image_name = "gun"
    with sm:
        smach.StateMachine.add('SURVEY_octagon_1', SurveyOctagonTask(image_name, 20, 0.5, octagon_euler_position),
                               transitions={
            'done': 'MOVE_TO_octagon_1'
        })

        smach.StateMachine.add('MOVE_TO_octagon_1', MoveToPoseLocalTask(*octagon_euler_position, listener),
                               transitions={
            'done': 'SURVEY_octagon_2'
        })

        smach.StateMachine.add('SURVEY_octagon_2', SurveyOctagonTask(image_name, 20, 0.5, octagon_euler_position),
                               transitions={
            'done': 'MOVE_TO_octagon_2'
        })

        smach.StateMachine.add('MOVE_TO_octagon_2', MoveToPoseLocalTask(*octagon_euler_position, listener),
                               transitions={
            'done': 'SURVEY_octagon_3'
        })

        smach.StateMachine.add('SURVEY_octagon_3', SurveyOctagonTask(image_name, 20, 1, octagon_euler_position),
                               transitions={
            'done': 'MOVE_TO_octagon_3'
        })

        smach.StateMachine.add('MOVE_TO_octagon_3', MoveToPoseLocalTask(*octagon_euler_position, listener),
                               transitions={
            'done': 'MOVE_AWAY_FROM_octagon'
        })

        smach.StateMachine.add('MOVE_AWAY_FROM_octagon', MoveToPoseLocalTask(0, 0, 4, 0, 0, 0, listener),
                               transitions={
            'done': 'succeeded'
        })

    return sm


def create_eyeball_octagon_task_sm(x_estimate, y_estimate):
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    listener = TransformListener()
    with sm:
        smach.StateMachine.add('MOVE_TO_OCTAGON', MoveToPoseLocalTask(x_estimate, y_estimate, 0, 0, 0, 0, listener),
                               transitions={
            'done': 'SURFACE'
        })

        smach.StateMachine.add('SURFACE', MoveToPoseLocalTask(0, 0, 20, 0, 0, 0, listener),
                               transitions={
            'done': 'succeeded'
        })

    return sm


class SurveyOctagonTask(Task):
    def __init__(self, object_name, time, ratio, output_euler_position):
        super(SurveyOctagonTask, self).__init__(["done"])
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
            octagon_vector = 0  # TODO: get octagon vector
            if octagon_vector is not None:
                captured_vectors.append(octagon_vector)
            total += millis
            rate.sleep()

        avg_x = mean([v[0] for v in captured_vectors]) * self.ratio
        avg_y = mean([v[1] for v in captured_vectors]) * self.ratio
        avg_z = (mean([v[2] for v in captured_vectors]) + 2) * self.ratio
        self.output_euler_position = [avg_x, avg_y, avg_z, 0, 0, 0]
        return "done"


if __name__ == '__main__':
    main()
