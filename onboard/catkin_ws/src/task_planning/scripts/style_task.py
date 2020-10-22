from task import Task
from geometry_msgs.msg import Pose, Quaternion, Twist, Point
from tf.transformations import quaternion_from_euler
import task_utils
import math
import rospy


class StyleTask(Task):
    def __init__(self, axis, power, angle = 2*math.pi, num_segments = 4):
        """Rotate using a given power past a certain angle and check that it reaches a num_segments

        Parameters:
        axis (string): orientation (x, y, z) of turn
        power (float): desired twist power of turn in (0, 1]
        angle

        """
        super(StyleTask, self).__init__()
        
        if num_segments < 3:
            raise Exception("num_segments must be >= 3 for StyleTask.")

        if power <= 0 or power > 1:
            raise Exception("power must be between (0, 1] for StyleTask.")
        
        self.axis = axis # either x, y, z
        self.num_segments = num_segments
        self.twist = Twist()
        self.seg_rads = angle / num_segments
        
        direction = angle / abs(angle)

        if axis == "x":
            self.twist.angular.x = power * direction
            self.q_angle = quaternion_from_euler(self.seg_rads, 0, 0)
        elif axis == "y":
            self.twist.angular.y = power * direction
            self.q_angle = quaternion_from_euler(0, self.seg_rads, 0)
        elif axis == "z":
            self.twist.angular.z = power * direction
            self.q_angle = quaternion_from_euler(0, 0, self.seg_rads)
    
    def _on_task_start(self):
        """Set some starting values for the style rotation."""
        self.current_segment = 0
        self.angle_pose = Pose(Point(), Quaternion(*self.q_angle))
        self.target_pose = task_utils.add_poses([self.state.pose.pose, self.angle_pose])
        self.starting_pose = self.state.pose.pose
        self.on_finish_segment = False

    def _on_task_run(self):
        """Go through the rotation using power control, checking that we hit certain segments as we go."""
        self.publish_desired_twist_power(self.twist)

        print("Now starting turn...")
        if not self.on_finish_segment and task_utils.at_pose(self.state.pose.pose, self.target_pose, float("inf"), self.seg_rads):
            self.current_segment += 1
            self.target_pose = task_utils.add_poses([self.target_pose, self.angle_pose])
            print("Now on segment " + self.current_segment)
        
        if not self.on_finish_segment and self.current_segment == self.num_segments:
            self.on_finish_segment = True
            self.target_pose = task_utils.add_poses([self.target_pose, self.angle_pose])
        
        if self.on_finish_segment and task_utils.at_pose(self.state.pose.pose, self.target_pose, float("inf"), 2 * self.seg_rads):
            self.publish_desired_twist_power(Twist())
            print("Turn Complete!\n")
            self.finish()
