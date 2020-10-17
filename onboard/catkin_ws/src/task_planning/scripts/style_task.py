from task import Task
from geometry_msgs.msg import Pose, Quaternion, Twist, Point
from tf.transformations import quaternion_from_euler
import task_utils
import math
import rospy


class StyleTask(Task):
    def __init__(self, axis, power, angle = 2*math.pi, num_segments = 4):
        super(StyleTask, self).__init__()

        self.twist_power = power
        self.axis = axis # either x, y, z
        self.num_segments = num_segments
        self.twist = Twist()

        if axis == "x":
            self.twist.angular.x = power
            self.q_angle = quaternion_from_euler(angle/num_segments, 0, 0)
        elif axis == "y":
            self.twist.angular.y = power
            self.q_angle = quaternion_from_euler(0, angle/num_segments, 0)
        elif axis == "z":
            self.twist.angular.z = power
            self.q_angle = quaternion_from_euler(0, 0, angle/num_segments)
    
    def _on_task_start(self):
        self.current_segment = 0
        self.angle_pose = Pose(Point(), self.q_angle)
        self.target_pose = task_utils.add_poses([self.state.pose.pose, self.angle_pose])
        
    def _on_task_run(self):
        self.publish_desired_twist_power(self, self.twist)

        if task_utils.at_pose(self.state.pose.pose, self.target_pose):
            self.current_segment += 1
            self.target_pose = task_utils.add_poses([self.target_pose, self.angle_pose])
        
        if self.current_segment == self.num_segments:
            self.publish_desired_twist_power(self, Twist())
            self.finish()
