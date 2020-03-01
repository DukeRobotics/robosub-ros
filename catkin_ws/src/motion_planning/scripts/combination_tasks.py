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
		self.all_finished = True
		for task in self.tasks:
			task.run()
			if not task.finished:
				self.all_finished = False

		if self.all_finished:
			self.finish()
			

class MasterSlaveTask(Task):

    def __init__(self, master, slave):
        super(MasterSlaveTask, self).__init__()
        self.master = master
        self.slave = slave

    def _on_task_run(self):
        if(self.master.finished):
            self.slave.finish()
            self.finish()

        self.master.run()
        self.slave.run()