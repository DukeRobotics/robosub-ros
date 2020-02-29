from task import Task

class ListTask(Task):
	"""Run a list of tasks sequentially"""

	def __init__(self, tasks, *args, **kwargs):
		super(ListTask, self).__init__(*args, **kwargs)

		self.tasks = tasks
		self.curr_index = 0

	def _on_task_run(self):
		if self.curr_index == len(self.tasks):
			self.finish()

		elif self.tasks[self.curr_index].finished:
			self.curr_index += 1

		else:
			self.tasks[self.curr_index].run()

class SimulTask(Task):
	"""Run a list of tasks simulataneously"""

	def __init__(self, tasks, *args, **kwargs):
		super(SimulTask, self).__init__(*args, **kwargs)

		self.tasks = tasks

	def _on_task_run(self):
		bool all_finished = True
		for task in tasks:
			if not task.finished:
				all_finished = False
				task.run()
		if all_finished:
			self.finish()