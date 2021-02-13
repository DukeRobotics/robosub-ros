from task import Task
from geometry_msgs.msg import Pose, Quaternion, Twist, Point, Vector3
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from move_tasks import AllocateVelocityLocalTask
import task_utils
import rospy


class MoveOneCVPointToAnotherTask(Task):
    """Move the robot such that a given point in a CV frame is moved to the specified location in the frame.
    method is either "rotate" or "strafe"
    camera is either "front" or "down"
    coordinates are scaled from 0.0 to 1.0 (e.g. x_curr=0.0 is the left side of the frame), and so is the tolerance
    """

    def __init__(self, cv_obstacle, x_target, y_target, method="rotate", camera="front", tolerance=0.1):
        super(MoveOneCVPointToAnotherTask, self).__init__()

        self.cv_obstacle = cv_obstacle
        self.x_target = x_target
        self.y_target = y_target
        self.method = method
        self.camera = camera
        self.tolerance = tolerance
        self.linear_constant = 0.5
        self.rotation_constant = 0.2

    def _on_task_start(self):
        pass

    def _on_task_run(self):  # better to use pose once we have depth info
        o = self.cv_data[self.cv_obstacle]
        self.x_curr = o.xmin + (o.xmax - o.xmin) / 2
        self.y_curr = o.ymin + (o.ymax - o.ymin) / 2
        x_diff = self.x_target - self.x_curr
        y_diff = self.y_target - self.y_curr

        if abs(x_diff) <= self.tolerance and abs(y_diff) <= self.tolerance:
            if self.vel_task:
                self.vel_task.finish()
            self.finish()
            return

        x_linear_vel = self.linear_constant * x_diff
        y_linear_vel = self.linear_constant * y_diff
        x_angular_vel = self.rotation_constant * x_diff
        y_angular_vel = self.rotation_constant * y_diff

        if self.vel_task:
            self.vel_task.finish()

        if self.method == "rotate" and self.camera == "front":
            self.vel_task = AllocateVelocityLocalTask(0, 0, 0, 0, -y_angular_vel, x_angular_vel)
        elif self.method == "rotate" and self.camera == "down":
            self.vel_task = AllocateVelocityLocalTask(0, 0, 0, x_angular_vel, -y_angular_vel, 0)
        elif self.method == "strafe" and self.camera == "front":
            self.vel_task = AllocateVelocityLocalTask(0, x_linear_vel, y_linear_vel, 0, 0, 0)
        elif self.method == "strafe" and self.camera == "down":
            self.vel_task = AllocateVelocityLocalTask(y_linear_vel, x_linear_vel, 0, 0, 0, 0)

        if self.vel_task:
            self.vel_task.run()
