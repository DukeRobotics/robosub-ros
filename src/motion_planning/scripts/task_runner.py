import rospy
from task import Task
from competition_task import CompetitionTask

class TaskRunner(object):

	RATE = 30 # Hz

	def __init__(self):
		self.task = CompetitionTask()
		self.rate = rospy.Rate(self.RATE)

	def start(self):
		rospy.init_node("task_planning")
		
		while not task.finished:
			self.task.run()
			self.rate.sleep()


TaskRunner().start()