#!/usr/bin/env python

from combination_tasks import ListTask
from move_tasks import MoveToPoseGlobalTask
from move_tasks import MoveToPoseLocalTask
from task import Task
import smach
import rospy
import gate_task
from tf import TransformListener
import math
import time
from custom_msgs.msg import ThrusterSpeeds


def main():
    rospy.init_node('gate_task')

    jank_prequal_with_turn()

    # sm = create_prequal_task_sm()
    # time.sleep(2)
    # # Execute SMACH plan
    # outcome = sm.execute()


SPEED = 80
DIVE_SPEED = 0
DEPTH_HOLD_SPEED = 0
TURN_TIME = 6
DIAMOND_TIME = 4
LONG_TIME = 5

# NOTE: The third sign is not what I expected, looking at the robot
# We need to figure out what "flipped" means
# It seems to mean counterclockwise
FORWARD_THRUST = [-SPEED, -SPEED, -SPEED, SPEED, DEPTH_HOLD_SPEED, -
                  DEPTH_HOLD_SPEED, -DEPTH_HOLD_SPEED, DEPTH_HOLD_SPEED]
BACKWARD_THRUST = [SPEED, SPEED, SPEED, -SPEED, DEPTH_HOLD_SPEED, -
                   DEPTH_HOLD_SPEED, -DEPTH_HOLD_SPEED, DEPTH_HOLD_SPEED]
DOWNWARD_THRUST = [0, 0, 0, 0, DIVE_SPEED, -DIVE_SPEED, -DIVE_SPEED, DIVE_SPEED]
FORWARD_LEFT_THRUST = [-SPEED, 0, 0, SPEED, DEPTH_HOLD_SPEED, -DEPTH_HOLD_SPEED, -DEPTH_HOLD_SPEED, DEPTH_HOLD_SPEED]
FORWARD_RIGHT_THRUST = [0, -SPEED, -SPEED, 0, DEPTH_HOLD_SPEED, -DEPTH_HOLD_SPEED, -DEPTH_HOLD_SPEED, DEPTH_HOLD_SPEED]
BACKWARD_RIGHT_THRUST = [SPEED, 0, 0, -SPEED, DEPTH_HOLD_SPEED, -DEPTH_HOLD_SPEED, -DEPTH_HOLD_SPEED, DEPTH_HOLD_SPEED]
BACKWARD_LEFT_THRUST = [0, SPEED, SPEED, 0, DEPTH_HOLD_SPEED, -DEPTH_HOLD_SPEED, -DEPTH_HOLD_SPEED, DEPTH_HOLD_SPEED]
SPIN_LEFT_THRUST = [-SPEED, SPEED, SPEED, -SPEED, DEPTH_HOLD_SPEED, -
                    DEPTH_HOLD_SPEED, -DEPTH_HOLD_SPEED, DEPTH_HOLD_SPEED]
SPIN_RIGHT_THRUST = [SPEED, -SPEED, -SPEED, SPEED, DEPTH_HOLD_SPEED, -
                     DEPTH_HOLD_SPEED, -DEPTH_HOLD_SPEED, DEPTH_HOLD_SPEED]


def move_with_thrust_for_seconds(pub, rate, thrust, seconds):
    for i in range(seconds * 10):
        t = ThrusterSpeeds()
        t.speeds = thrust
        pub.publish(t)
        rate.sleep()


def jank_prequal_with_turn():
    pub = rospy.Publisher('/offboard/thruster_speeds', ThrusterSpeeds, queue_size=3)
    rate = rospy.Rate(10)
    # time.sleep(10)
    # Down for 1 second
    move_with_thrust_for_seconds(pub, rate, DOWNWARD_THRUST, 1)
    # Forward for 3 seconds
    move_with_thrust_for_seconds(pub, rate, FORWARD_THRUST, LONG_TIME)
    # Spin left for 1 second
    move_with_thrust_for_seconds(pub, rate, SPIN_LEFT_THRUST, 1)
    # Forward for 2 seconds
    move_with_thrust_for_seconds(pub, rate, FORWARD_THRUST, DIAMOND_TIME)
    # Spin right for 1 second
    move_with_thrust_for_seconds(pub, rate, SPIN_RIGHT_THRUST, TURN_TIME)
    # Forward for 2 seconds
    move_with_thrust_for_seconds(pub, rate, FORWARD_THRUST, DIAMOND_TIME)
    # Spin right for 1 second
    move_with_thrust_for_seconds(pub, rate, SPIN_RIGHT_THRUST, TURN_TIME)
    # Forward for 2 seconds
    move_with_thrust_for_seconds(pub, rate, FORWARD_THRUST, DIAMOND_TIME)
    # Spin right for 1 second
    move_with_thrust_for_seconds(pub, rate, SPIN_RIGHT_THRUST, TURN_TIME)
    # Forward for 2 seconds
    move_with_thrust_for_seconds(pub, rate, FORWARD_THRUST, DIAMOND_TIME)
    # Spin left for 1 second
    move_with_thrust_for_seconds(pub, rate, SPIN_LEFT_THRUST, 1)
    # Forward for 3 seconds
    move_with_thrust_for_seconds(pub, rate, FORWARD_THRUST, LONG_TIME)


def jank_prequal():
    pub = rospy.Publisher('/offboard/thruster_speeds', ThrusterSpeeds, queue_size=3)
    rate = rospy.Rate(10)
    time.sleep(10)
    # Down for 1 second
    for i in range(10):
        pub.publish(ThrusterSpeeds(DOWNWARD_THRUST))
        rate.sleep()
    # Forward for 3 seconds
    for i in range(30):
        pub.publish(ThrusterSpeeds(FORWARD_THRUST))
        rate.sleep()
    # Forward-left for 2 seconds
    for i in range(20):
        pub.publish(ThrusterSpeeds(FORWARD_LEFT_THRUST))
        rate.sleep()
    # Forward-right for 2 seconds
    for i in range(20):
        pub.publish(ThrusterSpeeds(FORWARD_RIGHT_THRUST))
        rate.sleep()
    # Backward-right for 2 seconds
    for i in range(20):
        pub.publish(ThrusterSpeeds(BACKWARD_RIGHT_THRUST))
        rate.sleep()
    # Backward-left for 2 seconds
    for i in range(20):
        pub.publish(ThrusterSpeeds(BACKWARD_LEFT_THRUST))
        rate.sleep()
    # Backward for 3 seconds
    for i in range(30):
        pub.publish(ThrusterSpeeds(BACKWARD_THRUST))
        rate.sleep()


def create_prequal_task_sm():
    sm = smach.StateMachine(outcomes=['prequal_task_succeeded', 'prequal_task_failed'])
    listener = TransformListener()
    gate_sm = gate_task.create_gate_task_sm()

    DEPTH = -3
    STRETCH_LENGTH = 4
    LOOP_SIZE = 2

    with sm:
        # smach.StateMachine.add('GO_THROUGH_GATE', gate_sm,
        #                        transitions={
        #                            'gate_task_succeeded': 'APPROACH_MARKER',
        #                            'gate_task_failed': 'prequal_task_failed'})
        smach.StateMachine.add('SUBMERGE', MoveToPoseLocalTask(0, 0, DEPTH, 0, 0, 0, listener),
                               transitions={
                                   'done': 'LEFT_OF_MARKER'})
        smach.StateMachine.add('APPROACH_MARKER', MoveToPoseLocalTask(STRETCH_LENGTH, 0, 0, 0, 0, 0, listener),
                               transitions={
                                   'done': 'LEFT_OF_MARKER'})
        smach.StateMachine.add('LEFT_OF_MARKER', MoveToPoseLocalTask(LOOP_SIZE, LOOP_SIZE, 0, 0, 0, 0, listener),
                               transitions={
                                   'done': 'BEHIND_MARKER'})
        smach.StateMachine.add('BEHIND_MARKER', MoveToPoseLocalTask(LOOP_SIZE, -LOOP_SIZE, 0, 0, 0, 0, listener),
                               transitions={
                                   'done': 'RIGHT_OF_MARKER'})
        smach.StateMachine.add('RIGHT_OF_MARKER', MoveToPoseLocalTask(-LOOP_SIZE, -LOOP_SIZE, 0, 0, 0, 0, listener),
                               transitions={
                                   'done': 'BACK_TO_FRONT_OF_MARKER'})
        smach.StateMachine.add('BACK_TO_FRONT_OF_MARKER', MoveToPoseLocalTask(-LOOP_SIZE, LOOP_SIZE,
                               0, 0, 0, math.pi, listener), transitions={'done': 'BACK_THROUGH_GATE'})
        smach.StateMachine.add('BACK_THROUGH_GATE', MoveToPoseLocalTask(-STRETCH_LENGTH, 0, 0, 0, 0, 0, listener),
                               transitions={
                                   'done': 'prequal_task_succeeded'})
        # smach.StateMachine.add('BACK_THROUGH_GATE', gate_sm,
        #                        transitions={
        #                            'gate_task_succeeded': 'prequal_task_succeeded',
        #                            'gate_task_failed': 'prequal_task_failed'})

    return sm


if __name__ == '__main__':
    main()


# class PreQualGlobalTask(Task):
#     """
#     Task to complete prequalifying run by moving to global poses
#     """

#     POSE1 = [0, 0, 0, 0, 0, 0]
#     POSE2 = [0, 0, 0, 0, 0, 0]
#     POSE3 = [0, 0, 0, 0, 0, 0]
#     POSE4 = [0, 0, 0, 0, 0, 0]
#     POSE5 = [0, 0, 0, 0, 0, 0]

#     def __init__(self):
#         super(PreQualGlobalTask, self).__init__()

#         self.list_task = ListTask([MoveToPoseGlobalTask(*self.POSE1),
#                                    MoveToPoseGlobalTask(*self.POSE2),
#                                    MoveToPoseGlobalTask(*self.POSE3),
#                                    MoveToPoseGlobalTask(*self.POSE4),
#                                    MoveToPoseGlobalTask(*self.POSE5)])

#     def _on_task_run(self):
#         self.list_task.run()

#         if self.list_task.finished:
#             self.finish()


# class PreQualLocalTask(Task):
#     """
#     Task to complete prequalifying run by moving to local poses
#     """

#     POSE1 = [0, 0, 0, 0, 0, 0]
#     POSE2 = [0, 0, 0, 0, 0, 0]
#     POSE3 = [0, 0, 0, 0, 0, 0]
#     POSE4 = [0, 0, 0, 0, 0, 0]
#     POSE5 = [0, 0, 0, 0, 0, 0]

#     def __init__(self):
#         super(PreQualLocalTask, self).__init__()

#         self.list_task = ListTask([MoveToPoseLocalTask(*self.POSE1),
#                                    MoveToPoseLocalTask(*self.POSE2),
#                                    MoveToPoseLocalTask(*self.POSE3),
#                                    MoveToPoseLocalTask(*self.POSE4),
#                                    MoveToPoseLocalTask(*self.POSE5)])

#     def _on_task_run(self):
#         self.list_task.run()

#         if self.list_task.finished:
#             self.finish()
