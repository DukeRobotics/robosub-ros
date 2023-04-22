#!/usr/bin/env python3

import rospy
import smach
import tf
from smach_test import controls_testing
from interface.controls import ControlsInterface
from buoy_task import BuoyTask
from interface.cv import CVInterface
from move_tasks import MoveToPoseGlobalTask
import random


class TaskRunner(smach.StateMachine):
    RATE = 30  # Hz

    def __init__(self):
        super(TaskRunner, self).__init__(outcomes=['done'])
        rospy.init_node("task_planning")
        self.listener = tf.TransformListener()
        self.controls = ControlsInterface(self.listener)
        self.cv = CVInterface()
        
        self.x, self.y = random.randint(-10, 10), random.randint(-10, 10)

        with self:
            smach.StateMachine.add('TEST', BuoyTask(self.listener, self.controls, self.cv),
                                   transitions={'done': 'done'})
            # smach.StateMachine.add('TEST', MoveToPoseGlobalTask(self.x, self.y, 0, 0, 0, 0, self.controls),
            #                        transitions={'done': 'done', 'continue': 'TEST'})
            # smach.StateMachine.add('TEST', controls_testing(self.controls, self.listener),
            #                        transitions={'done': 'done'})

    def execute(self):
        rospy.loginfo("Waiting for transform listener")
        self.listener.waitForTransform('odom', 'base_link', rospy.Time(), rospy.Duration(15))
        super(TaskRunner, self).execute()


def main():
    try:
        TaskRunner().execute()
    except rospy.ROSInterruptException:
        pass
    # from geometry_msgs.msg import Pose, Quaternion, Point
    # from tf.transformations import quaternion_from_euler
    # from custom_msgs.msg import ControlsDesiredPoseAction, ControlsDesiredPoseGoal
    # import actionlib
    
    # DESIRED_POSE_ACTION = 'controls/desired_pose'
    # DESIRED_TWIST_ACTION = 'controls/desired_twist'
    
    # desired_pose_client = actionlib.SimpleActionClient(
    # DESIRED_POSE_ACTION, ControlsDesiredPoseAction)
    
    # x, y, z = 5, 0, 0
    # roll, pitch, yaw = 0, 0, 0
    
    # desired_pose = Pose()
    # desired_pose.position = Point(x=x, y=y, z=z)
    # desired_pose.orientation = Quaternion(
    #     *
    #     quaternion_from_euler(
    #         roll,
    #         pitch,
    #         yaw))
    
    # desired_pose_client.send_goal(ControlsDesiredPoseGoal(pose=desired_pose))
    


if __name__ == '__main__':
    main()
