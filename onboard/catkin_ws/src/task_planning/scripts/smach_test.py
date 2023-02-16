#!/usr/bin/env python

import rospy
import smach
import random
from move_tasks import MoveToPoseGlobalTask, MoveToPoseLocalTask, AllocateVelocityLocalTask
import cv_tasks
from time import sleep
from geometry_msgs.msg import Vector3
from tf import TransformListener
# from interface.controls import ControlsInterface
from interface.cv import CVInterface

# define state Foo


def main():
    rospy.init_node('smach_test')
    listener = TransformListener()
    # controls = ControlsInterface(listener)
    cv = CVInterface(listener)

    sm = buoy_cv_test(cv)

    rospy.loginfo("Waiting for transform listener")
    listener.waitForTransform('odom', 'base_link', rospy.Time(), rospy.Duration(15))

    # Execute SMACH plan
    sm.execute()


def buoy_cv_test(cv):
    sm = smach.StateMachine(outcomes=['done'])

    with sm:
        smach.StateMachine.add('FindBuoy', cv_tasks.ObjectCoordsTask('bootleggerbuoy', cv),
                               transitions={
                                    'invalid': 'FindBuoy',
                                    'valid': 'PrintBuoy'
                                })
        smach.StateMachine.add('PrintBuoy', PrintUserDataTask("coords"),
                               transitions={
                                    'done': 'done'
                                })

    return sm


class RandomizeOutputPose(smach.State):
    def __init__(self):
        super(RandomizeOutputPose, self).__init__(
            outcomes=["done"],
            output_keys=['x', 'y', 'z', 'roll', 'pitch', 'yaw'])

    def execute(self, userdata):
        sleep(1)
        if self.preempt_requested():
            self.service_preempt()
            return 'done'
        userdata.x = random.random() * 10
        userdata.y = random.random() * 10
        userdata.z = random.random() * 10

        userdata.roll = 0
        userdata.pitch = 0
        userdata.yaw = 0

        return "done"


def object_passing():
    sm = smach.StateMachine(outcomes=['done'])

    with sm:
        smach.StateMachine.add('calc', OutputVector3Task(Vector3(3, 5, 3)),
                               transitions={'done': 'print'})

        smach.StateMachine.add('print', PrintUserDataTask(),
                               transitions={'done': 'done'})

    return sm


def controls_testing(controls, listener):
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['done'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Move1', MoveToPoseLocalTask(2, 0, 0, 0, 0, 0, controls, listener),
                               transitions={'done': 'finish', 'continue': 'Move1'})
        # left square
        #smach.StateMachine.add('MoveLeft2', MoveToPoseLocalTask(0, -1.5, 0, 0, 0, 0, controls, listener),
        #                       transitions={'done': 'MoveLeft3', 'continue': 'MoveLeft2'})
        #smach.StateMachine.add('MoveLeft3', MoveToPoseLocalTask(1.5, 0, 0, 0, 0, 0, controls, listener),
        #                       transitions={'done': 'MoveLeft4', 'continue': 'MoveLeft3'})
        #smach.StateMachine.add('MoveLeft4', MoveToPoseLocalTask(0, 1.5, 0, 0, 0, 0, controls, listener),
        #                       transitions={'done': 'finish', 'continue': 'MoveLeft4'})
        # right square
        #smach.StateMachine.add('MoveRight2', MoveToPoseGlobalTask(2, -2, 0, 0, 0, 0, controls),
        #                     transitions={'done': 'MoveRight3', 'continue': 'MoveRight2'})
        #smach.StateMachine.add('MoveRight3', MoveToPoseGlobalTask(0, -2, 0, 0, 0, 0, controls),
        #                      transitions={'done': 'MoveRight4', 'continue': 'MoveRight3'})
        #smach.StateMachine.add('MoveRight4', MoveToPoseGlobalTask(0, 0, 0, 0, 0, 0, controls),
        #                       transitions={'done': 'finish', 'continue': 'MoveRight4'})

    return sm


def concurrency(controls):
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['finish'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Move1', MoveToPoseGlobalTask(2, 0, 0, 0, 0, 0, controls),
                               transitions={'done': 'ConcurrentMove2', 'continue': 'Move1'})
        cc = smach.Concurrence(outcomes=['done'],
                               default_outcome='done',
                               outcome_map={'done': {'Move2': 'done', 'Log': 'done'}})
        with cc:
            smach.Concurrence.add('Move2', MoveToPoseGlobalTask(2, 2, 0, 0, 0, 0, controls))
            smach.Concurrence.add('Log', LogSomethingUseful())

        smach.StateMachine.add('ConcurrentMove2', cc, transitions={'done': 'Move3'})

        smach.StateMachine.add('Move3', MoveToPoseGlobalTask(0, 2, 0, 0, 0, 0, controls),
                               transitions={'done': 'Move4', 'continue': 'Move3'})
        smach.StateMachine.add('Move4', MoveToPoseGlobalTask(0, 0, 0, 0, 0, 0, controls),
                               transitions={'done': 'finish', 'continue': 'Move4'})

    return sm


def decision_making(controls):
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['finish'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Move1', MoveToPoseGlobalTask(2, 0, 0, 0, 0, 0, controls),
                               transitions={'done': 'choice', 'continue': 'Move1'})
        smach.StateMachine.add("choice", RandomChoice(2),
                               transitions={'0': 'MoveLeft2', '1': 'MoveRight2'})
        # left square
        smach.StateMachine.add('MoveLeft2', MoveToPoseGlobalTask(2, 2, 0, 0, 0, 0, controls),
                               transitions={'done': 'MoveLeft3', 'continue': 'MoveLeft2'})
        smach.StateMachine.add('MoveLeft3', MoveToPoseGlobalTask(0, 2, 0, 0, 0, 0, controls),
                               transitions={'done': 'MoveLeft4', 'continue': 'MoveLeft3'})
        smach.StateMachine.add('MoveLeft4', MoveToPoseGlobalTask(0, 0, 0, 0, 0, 0, controls),
                               transitions={'done': 'finish', 'continue': 'MoveLeft4'})
        # right square
        smach.StateMachine.add('MoveRight2', MoveToPoseGlobalTask(2, -2, 0, 0, 0, 0, controls),
                               transitions={'done': 'MoveRight3', 'continue': 'MoveRight2'})
        smach.StateMachine.add('MoveRight3', MoveToPoseGlobalTask(0, -2, 0, 0, 0, 0, controls),
                               transitions={'done': 'MoveRight4', 'continue': 'MoveRight3'})
        smach.StateMachine.add('MoveRight4', MoveToPoseGlobalTask(0, 0, 0, 0, 0, 0, controls),
                               transitions={'done': 'finish', 'continue': 'MoveRight4'})

    return sm


def simple_move_test(controls, listener):
    sm = smach.StateMachine(outcomes=['done'])

    with sm:
        smach.StateMachine.add("Move", MoveToPoseLocalTask(-2, 0, 0, 0, 0, 0, controls, listener),
                               transitions={
                                    'continue': 'Move',
                                    'done': 'Surface'
                                })
        smach.StateMachine.add("Surface", AllocateVelocityLocalTask(0, 0, 1, 0, 0, 0, controls),
                               transitions={
                                    'done': 'Surface'
                                })

    return sm


# Test if userdata output by a task can be accessed even with another task in between
def userdata_passthrough():
    sm = smach.StateMachine(outcomes=['done'])

    with sm:
        smach.StateMachine.add("Output", OutputVector3Task(Vector3(1, 2, 3)),
                               transitions={
                                    'done': 'Print'
                                })
        smach.StateMachine.add("Print", PrintUserDataTask(),
                               transitions={
                                    'done': 'Print2'
                                })
        smach.StateMachine.add("Print2", PrintUserDataTask(),
                               transitions={
                                    'done': 'done'
                                })

    return sm


class OutputVector3Task(smach.State):
    def __init__(self, vec):
        super().__init__(outcomes=["done"], output_keys=['vec'])
        self.vec = vec

    def execute(self, userdata):
        userdata.vec = self.vec
        return "done"


class PrintUserDataTask(smach.State):
    def __init__(self, key='vec'):
        super().__init__(outcomes=["done"], input_keys=[key])
        self.key = key

    def execute(self, userdata):
        print(userdata[self.key])
        return "done"


class RandomChoice(smach.State):
    """Randomly chooses a number"""

    def __init__(self, options):
        super(RandomChoice, self).__init__(outcomes=[str(i) for i in range(options)])

        self.options = options

    def execute(self, userdata):
        res = str(random.randint(0, self.options))
        rospy.loginfo(res)
        return res


class LogSomethingUseful(smach.State):
    """Logs Stuff"""

    def __init__(self):
        super(LogSomethingUseful, self).__init__(outcomes=["done"])

    def execute(self, userdata):
        rate = rospy.Rate(1)
        for i in range(40):
            rospy.loginfo("something important")
            rate.sleep()
        return "done"


if __name__ == '__main__':
    main()
