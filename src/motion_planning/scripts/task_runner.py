import rospy
from task import Task
from competition_task import CompetitionTask
from task_state import TaskState

class TaskRunner(object):

	RATE = 30 # Hz
	

	def __init__(self):
		self.rate = rospy.Rate(self.RATE)
		self.task_state = TaskState()
        self.competition_task = CompetitionTask()

	def start(self):
		rospy.init_node("task_planning")

		# Wait to receive state before running any tasks
		while not self.task_state.state:
			self.rate.sleep()
		
		while not self.competition_task.finished:
			self.competition_task.run()
			self.rate.sleep()


TaskRunner().start()