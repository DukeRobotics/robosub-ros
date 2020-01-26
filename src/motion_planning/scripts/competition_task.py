from template_tasks import *

GateTask = ListTask([
	# Submerge
	MoveToPoseLocalTask(0, 0, -2, 0, 0, 0),
	# Go straight
	MoveToPoseLocalTask(10, 0, 0, 0, 0, 0)
	])
CompetitionTask = ListTask([GateTask])
