from task_state import TaskState
import dependency_injector.providers as providers
import smach

from abc import ABCMeta, abstractmethod


class Task(smach.State):
    """High level task that represents some function"""

    task_state_provider = providers.Singleton(TaskState)

    def __init__(self, outcomes, input_keys=[], output_keys=[], io_keys=[]):
        """
        Create a Task.

        """
        smach.State.__init__(self, outcomes, input_keys, output_keys, io_keys)

        self.task_state = self.task_state_provider()
        self.start_time = None
        self.initial_state = None
        self.output = {}

    @property
    def state(self):
        """Wrap task_state.state with just the state property"""
        return self.task_state.state

    @property
    def cv_data(self):
        return self.task_state.cv_data

    @abstractmethod
    def run(self, userdata):
        """Try to complete the task

        Override this method for a particular task."""
        pass

    def execute(self, userdata):
        self.initial_state = self.state
        return self.run(userdata)

    def publish_desired_pose_global(self, pose):
        self.task_state.desired_pose_global_publisher.publish(pose)

    def publish_desired_power(self, twist_power):
        self.task_state.desired_twist_power_publisher.publish(twist_power)

    def publish_desired_twist(self, twist_velocity):
        self.task_state.desired_twist_velocity_publisher.publish(twist_velocity)
