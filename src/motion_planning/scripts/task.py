import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist

from abc import ABC, abstractmethod

class Task(ABC):
    """High level task that represents some function"""

    STATE_TOPIC = 'state'
    DESIRED_POSE_TOPIC = 'controls/desired_pose_global'
    DESIRED_TWIST_GLOBAL_TOPIC = 'controls/desired_twist_global'
    DESIRED_TWIST_LOCAL_TOPIC = 'controls/desired_twist_local'
    

    def __init__(self):
        self.start_time = None
        self.finished = False
        self.initial_state = None
        self.started = False
        self.state = None

        self.state_listener = rospy.Subscriber(self.STATE_TOPIC, Odometry, self._on_receive_state)
        self.desired_pose_global_publisher = rospy.Publisher(self.DESIRED_POSE_TOPIC, Pose, queue_size=5)
        self.desired_twist_global_publisher = rospy.Publisher(self.DESIRED_TWIST_GLOBAL_TOPIC, Twist, queue_size=5)
        self.desired_twist_local_publisher = rospy.Publisher(self.DESIRED_TWIST_LOCAL_TOPIC, Twist, queue_size=5)

    def _initialize(self):
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
    def _task_run(self):
        """Override this method for a particular task"""
        pass

    def _task_init(self):
        """Override this method with code that must be run once, at the beginning
        of the task

        Note: Not marked as abstract method because custom init is not always necessary
        for subclasses"""
        pass
    
    def finish(self):
        """Mark the task as finished"""
        self.finished = True

    def publish_desired_pose_global(self, pose):
        self.desired_pose_global_publisher.publish(pose)

    def publish_desired_twist_global(self, twist):
        self.desired_twist_global_publisher.publish(twist)

    def publish_desired_twist_local(self, twist):
        self.desired_twist_local_publisher.publish(twist)

    def _on_receive_state(self, state):
        """Receive the state, update initial_state if it is empty
        and the task is running"""
        self.state = state
        if self.initial_state is None and not self.started:
            self.initial_state = state
