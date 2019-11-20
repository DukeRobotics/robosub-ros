import rospy

class Task(object):
    """High level task that represents some function"""

    STATE_TOPIC = 'state'

    def __init__(self):
        self.start_time = None
        self.finished = False
        self.initial_state = None
        self.started = False
        self.state_listener = rospy.Publisher(self.STATE_TOPIC, Odometry, self._on_receive_state)
        self.state = None

    def _initialize(self):
        """Should be called when the task runs for the first time"""
        self.start_time = rospy.get_rostime()
        self.initial_state = self.state if self.state else None

    def run(self):
        """Run the task. This should be called by motion planning, and
        will call run_task, which is the task specific run method"""
        if not self.started:
            self._initialize()
            self.started = True
        
        if initial_state is None:
            return
        
        self.run_task()
    
    def _run_task(self):
        """Override this method for a particular task"""
        pass
    
    def finish(self):
        """Mark the task as finished"""
        self.finished = True

    def _on_receive_state(self, state):
        """Receive the state, update initial_state if it is empty
        and the task is running"""
        self.state = state
        if self.initial_state is None and self.start_time is not None:
            self.initial_state = state
