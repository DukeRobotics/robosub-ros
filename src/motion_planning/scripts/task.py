import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from task_state import TaskState

from abc import ABC, abstractmethod

class Task(ABC):
    """High level task that represents some function"""
    

    def __init__(self, *args, **kwargs):
        """ 
        Create a Task.

        **kwargs:
        task_state (TaskState): A TaskState object that contains the state listener and controls publishers
        """
        if 'task_state' not in kwargs:
            raise ValueError("task_state must be passed in **kwargs and must not be None")
        elif not isinstance(kwargs.get('task_state'), TaskState):
            raise ValueError("task_state must be an instance of TaskState")

        self.task_state = kwargs.get('task_state')

        self.start_time = None
        self.finished = False
        self.initial_state = None
        self.started = False

    @property
    def state(self):
        """Wrap task_state.state with just the state property"""
        return self.task_state.state


    def _on_task_start_default(self):
        """Should be called when the task runs for the first time"""
        self.start_time = rospy.get_rostime()
        self.initial_state = self.state
        self._task_init()

    def run(self):
        """Run the task. This should be called by the task planner, and
        will call _task_run, which is the task specific run method"""

        if self.finished:
            return

        if not self.started:
            self._initialize()
            self.started = True
        
        if self.initial_state is None:
            return
        
        self._task_run()
    
    @abstractmethod
    def _on_task_run(self):
        """Try to complete the task

        Override this method for a particular task."""
        pass

    def _on_task_start(self):
        """Run one time when a task starts running

        Override this method with code that must be run once, at the beginning
        of the task

        Note: Not marked as abstract method because custom init is not always necessary
        for subclasses"""
        pass
    
    def finish(self):
        """Mark the task as finished"""
        self.finished = True

    def publish_desired_pose_global(self, pose):
        self.task_state.desired_pose_global_publisher.publish(pose)

    def publish_desired_twist_power(self, twist_power):
        self.task_state.desired_twist_power_publisher.publish(twist_power)
