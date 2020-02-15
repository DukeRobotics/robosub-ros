from task_state import TaskState
import dependency_injector.providers as providers

from abc import ABCMeta, abstractmethod


class Task:
    """High level task that represents some function"""

    __metaclass__ = ABCMeta

    task_state_provider = providers.Singleton(TaskState)

    def __init__(self, *args, **kwargs):
        """
        Create a Task.

        """

        self.task_state = self.task_state_provider()

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
        self.initial_state = self.state

        self._on_task_start()

    def run(self):
        """Run the task. This should be called by the task planner, and
        will call _task_run, which is the task specific run method"""
        if self.finished or not self.state:
            return

        if not self.started:
            self._on_task_start_default()
            self.started = True
<<<<<<< HEAD:onboard/catkin_ws/src/motion_planning/scripts/task.py

        self._on_task_run()

=======
        
        while not self.initial_state:
            pass
        self._on_task_run()
    
>>>>>>> don't run task until state is initialized:catkin_ws/src/motion_planning/scripts/task.py
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
