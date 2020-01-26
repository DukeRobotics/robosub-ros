import rospy
from task import Task
from template_tasks import *
from competition_task import CompetitionTask

class TaskRunner(object):

	RATE = 30 # Hz
	STATE_TOPIC = 'state'
    DESIRED_POSE_TOPIC = 'controls/desired_pose_global'
    DESIRED_TWIST_GLOBAL_TOPIC = 'controls/desired_twist_global'
    DESIRED_TWIST_LOCAL_TOPIC = 'controls/desired_twist_local'

	# Initialise static members that will hold the publishers and listeners
	STATE_LISTENER = None
	DESIRED_POSE_GLOBAL_PUBLISHER = None
	DESIRED_TWIST_GLOBAL_PUBLISHER = None
	DESIRED_TWIST_LOCAL_PUBLISHER = None

	def __init__(self):
		self.task = CompetitionTask
		self.rate = rospy.Rate(self.RATE)
		self.state = None

        self.STATE_LISTENER = rospy.Subscriber(self.STATE_TOPIC, Odometry, self._on_receive_state)
        self.DESIRED_POSE_GLOBAL_PUBLISHER = rospy.Publisher(self.DESIRED_POSE_TOPIC, Pose, queue_size=5)
        self.DESIRED_TWIST_GLOBAL_PUBLISHER = rospy.Publisher(self.DESIRED_TWIST_GLOBAL_TOPIC, Twist, queue_size=5)
        self.DESIRED_TWIST_LOCAL_PUBLISHER = rospy.Publisher(self.DESIRED_TWIST_LOCAL_TOPIC, Twist, queue_size=5)

	def start(self):
		rospy.init_node("task_planning")
		
		while not self.task.finished:
			self.task.run()
			self.rate.sleep()

    def _on_receive_state(self, state):
        """Receive the state, update initial_state if it is empty
        and the task is running"""
        self.state = state
        if self.initial_state is None and not self.started:
            self.initial_state = state

TaskRunner().start()