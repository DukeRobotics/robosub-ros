#!/usr/bin/env python3

import rospy
import smach
from geometry_msgs.msg import Pose, PoseStamped
from move_tasks import MoveToPoseLocalTask
from tf import TransformListener

"""smach.StateMachine.add('MoveLeft2', MoveToPoseLocalTask(2, 0, -1, 0, 0, 0),
                            transitions={'done':'MoveLeft3'})
        smach.StateMachine.add('MoveLeft3', MoveToPoseLocalTask(2, 0, 0, 0, 0, 0),
                            transitions={'done':'MoveLeft4'})
        smach.StateMachine.add('MoveLeft4', MoveToPoseLocalTask(2, 0, 1, 0, 0, 0),
                            transitions={'done':'finish'})"""


def main():
    rospy.init_node("task_planning")
    MoveLocally(2, 0, -1, 0, 0, 0, 0).publish_desired_pose_local()


def main2():
    rospy.init_node("task_planning")

    sm_top = smach.StateMachine(outcomes=['finish'])
    with sm_top:
        smach.StateMachine.add('MoveLeft2', MoveToPoseLocalTask(2, 0, -1, 0, 0, 0),
                               transitions={'done': 'MoveLeft3'})
        smach.StateMachine.add('MoveLeft3', MoveToPoseLocalTask(2, 0, 0, 0, 0, 0),
                               transitions={'done': 'MoveLeft4'})
        smach.StateMachine.add('MoveLeft4', MoveToPoseLocalTask(2, 0, 1, 0, 0, 0),
                               transitions={'done': 'finish'})

    try:
        sm_top.execute()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()

# define state Bas


class MoveLocally():
    def __init__(self, px, py, pz, ox, oy, oz, ow):
        self.px = px
        self.py = py
        self.pz = pz

        self.ox = ox
        self.oy = oy
        self.oz = oz
        self.ow = ow
        self.PUBLISHING_TOPIC_DESIRED_POSE = 'controls/desired_pose'
        self.listener = TransformListener()
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAS')
        rate = rospy.Rate(15)

        self.publish_desired_pose_local()

        while not rospy.is_shutdown():
            self._pub_desired_pose.publish(self.desired_pose_transformed)
            rate.sleep()

        return 'done'

    def publish_desired_pose_local(self):
        self._pub_desired_pose = rospy.Publisher(self.PUBLISHING_TOPIC_DESIRED_POSE, Pose, queue_size=3)

        self.desired_pose_local = Pose()
        self.desired_pose_local.position.x = self.px
        self.desired_pose_local.position.y = self.py
        self.desired_pose_local.position.z = self.pz
        self.desired_pose_local.orientation.x = self.ox
        self.desired_pose_local.orientation.y = self.oy
        self.desired_pose_local.orientation.z = self.oz
        self.desired_pose_local.orientation.w = self.ow

        pose_stamped = PoseStamped()
        pose_stamped.pose = self.desired_pose_local
        pose_stamped.header.frame_id = "base_link"
        self.desired_pose_transformed = self.listener.transformPose("odom", pose_stamped).pose

        rate = rospy.Rate(15)

        while not rospy.is_shutdown():
            self._pub_desired_pose.publish(self.desired_pose_transformed)
            rate.sleep()
