from task import Task
from geometry_msgs.msg import Pose, Quaternion, Twist, Point
from tf.transformations import quaternion_from_euler
import task_utils, move_tasks
import math
import rospy
from move_tasks import AllocateVelocityGlobalTask, AllocateVelocityLocalTask


class StyleTask(Task):
    def __init__(self, axis, speed, angle = 2*math.pi, num_segments = 4):
        """Rotate using a given power past a certain angle and check that it reaches a num_segments

        Parameters:
        axis (string): orientation (x, y, z) of turn
        power (float): desired twist power of turn in (0, 1]
        angle (float): desired angle of turn in radians
        num_segments (int): number of segments to check position durning turn (must be >= 3)

        """
        super(StyleTask, self).__init__()
        
        if num_segments < 3:
            raise Exception("num_segments must be >= 3 for StyleTask.")

        if speed <= 0 or speed > 1:
            raise Exception("power must be between (0, 1] for StyleTask.")
        
        self.num_segments = num_segments
        self.angle = angle
        self.seg_rads = angle / num_segments
        self.nintey_points = 100
        direction = angle / abs(angle)

        if axis == "x":
            self.rotate_task = move_tasks.AllocateVelocityTask(0,0,0,speed*direction,0,0)
            self.q_angle = quaternion_from_euler(self.seg_rads, 0, 0)
        elif axis == "y":
            self.rotate_task = move_tasks.AllocateVelocityTask(0,0,0,0,speed*direction,0)
            self.q_angle = quaternion_from_euler(0, self.seg_rads, 0)
        elif axis == "z":
            self.rotate_task = move_tasks.AllocateVelocityTask(0,0,0,0,0,speed*direction)
            self.q_angle = quaternion_from_euler(0, 0, self.seg_rads)
        else:
            raise Exception("axis must be \"x\", \"y\", or \"z\"")
    
    def _on_task_start(self):
        """Set some starting values for the style rotation."""
        self.current_segment = 0
        self.angle_pose = Pose(Point(), Quaternion(*self.q_angle))
        self.target_pose = task_utils.add_poses([self.state.pose.pose, self.angle_pose])
        self.starting_pose = self.state.pose.pose
        self.on_finish_segment = False
        self.output["rads_turned"] = 0.0
        self.output["points_scored"] = 0
        self.velocity_task = AllocateVelocityLocalTask(self.twist.linear.x,
                                                        self.twist.linear.y,
                                                        self.twist.linear.z,
                                                        self.twist.angular.x,
                                                        self.twist.angular.y,
                                                        self.twist.angular.z)
        rospy.loginfo("Now starting turn...")


    def _on_task_run(self):
        """Go through the rotation using power control, checking that we hit certain segments as we go."""
        # self.publish_desired_twist_power(self.twist)  # change to ALlocateVelocityTask
        # self.publish_desired_twist(self.twist.linear.x,
        #                              self.twist.linear.y,
        #                              self.twist.linear.z,
        #                              self.twist.angular.x,
        #                              self.twist.angular.y,
        #                              self.twist.angular.z
        #                              )
        self.velocity_task.run()

        if not self.on_finish_segment and task_utils.at_pose(self.state.pose.pose, self.target_pose, float("inf"), self.seg_rads / 2):
            self.current_segment += 1
            self.target_pose = task_utils.add_poses([self.target_pose, self.angle_pose])
            rospy.loginfo("Now on segment " + str(self.current_segment))
            self.output["rads_turned"] += self.seg_rads
            self.output["points_scored"] = int(self.output["rads_turned"] / (math.pi / 2)) * self.nintey_points
        
        if not self.on_finish_segment and self.current_segment == self.num_segments:
            self.on_finish_segment = True
        
        if self.on_finish_segment and task_utils.at_pose(self.state.pose.pose, self.target_pose, float("inf"), self.seg_rads):
            rospy.loginfo("Turn Complete!\n")
            self.output["rads_turned"] = self.angle
            self.output["points_scored"] = int(self.output["rads_turned"] / (math.pi / 2)) * self.nintey_points
            self.rotate_task.finish()
            self.finish()
