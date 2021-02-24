from task import Task
from geometry_msgs.msg import Pose, Quaternion, Twist, Point
from tf.transformations import quaternion_from_euler
import task_utils
import rospy

class StyleTask(Task):
    def __init__(self, axis, power):
        super(StyleTask, self).__init__(*args, **kwargs)

        self.twist_power = power
        self.axis = axis # either x, y, z

        self.twist = Twist()

        if(axis == "x"):
            self.twist.angular.x = power
        elif(axis == "y"):
            self.twist.angular.y = power
        elif(axis == "z"):
            self.twist.angular.z = power
        
    def _on_task_run(self):
        self.publish_desired_twist_power(self, self.twist)
        # TODO: Stop task once robot completed spin
