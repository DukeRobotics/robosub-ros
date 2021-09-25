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

    def __init__(self, cv_obstacle, x_target, y_target, method="strafe", camera="front", tolerance=0.1):
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
        if o.label == "none":
            # Obstacle not currently in view, or at least not detected by CV
            rospy.loginfo("Bounding box not in view! Stopping...")
            self.finish()
        
        x_curr = o.xmin + (o.xmax - o.xmin) / 2  # center of object
        y_curr = o.ymin + (o.ymax - o.ymin) / 2
        x_diff = self.x_target - x_curr  # difference to center of object
        y_diff = self.y_target - y_curr

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


class MoveOneCVBoxToAnotherTask(Task):
    """Move the robot such that a given bounding box in a CV frame is moved to the specified size and location in the frame.
    method is either "rotate" or "strafe"
    camera is either "front" or "down"
    coordinates are scaled from 0.0 to 1.0 (e.g. x_curr=0.0 is the left side of the frame), and so is the tolerance
    the center of the box will be aligned to x_target and y_target, and will match at least one of either w_target or h_target in dimension
    """

    def __init__(self, cv_obstacle, x_target, y_target, w_target, h_target, method="strafe", camera="front", tolerance=0.1):
        super(MoveOneCVPointToAnotherTask, self).__init__()

        self.cv_obstacle = cv_obstacle
        self.x_target = x_target
        self.y_target = y_target
        self.w_target = w_target
        self.h_target = h_target
        self.method = method
        self.camera = camera
        self.tolerance = tolerance
        self.linear_constant = 0.5
        self.rotation_constant = 0.2

    def _on_task_start(self):
        pass

    def _on_task_run(self):  # better to use pose once we have depth info
        o = self.cv_data[self.cv_obstacle]
        if o.label == "none":
            # Obstacle not currently in view, or at least not detected by CV
            rospy.loginfo("Bounding box f")
            self.finish()
        
        x_curr = o.xmin + (o.xmax - o.xmin) / 2  # center of object
        y_curr = o.ymin + (o.ymax - o.ymin) / 2
        x_diff = self.x_target - x_curr  # difference to center of object
        y_diff = self.y_target - y_curr

        w_curr = o.xmax - o.xmin
        h_curr = o.ymax - o.ymin
        w_diff = self.w_target - w_curr
        h_diff = self.h_target - h_curr

        target_area = self.w_target * self.h_target
        current_area = w_curr * h_curr

        # continue here:........... not done

        # determine if need to scale up or scale down box
        resize_box = current_area - target_area # probably want to add tolerance and change if statements accordingly

        if resize_box < 0: # current box smaller than target, so scale up
            # move to position and then scale up
                # check if position met (w or h is within tolerance)
                # then execute scale up
        elif resize_box > 0: # current box larger than target, so scale down
            # scale down then move to position
                # scale down and check if size requirement is met
                # then execute movement to put (w or H within tolerance)

        if (abs(x_diff) <= self.tolerance and abs(y_diff) <= self.tolerance and
            (abs(w_diff) < self.tolerance or abs(h_diff) < self.tolerance)):
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
