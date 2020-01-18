import rospy
from nav_msgs.msg import Odometry

class Task(object):
    """High level task that represents some function"""

    STATE_TOPIC = 'state'

    def __init__(self):
        self.start_time = None
        self.finished = False
        self.initial_state = None
        self.started = False
        self.state_listener = rospy.Subscriber(self.STATE_TOPIC, Odometry, self._on_receive_state, queue_size=5)
        self.state = None

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
    
    def _task_run(self):
        """Override this method for a particular task"""
        pass

    def _task_init(self):
        """Override this method with code that must be run once, at the beginning
        of the task"""
        pass
    
    def finish(self):
        """Mark the task as finished"""
        self.finished = True

    def _on_receive_state(self, state):
        """Receive the state, update initial_state if it is empty
        and the task is running"""
        self.state = state
        if self.initial_state is None and not self.started:
            self.initial_state = state
