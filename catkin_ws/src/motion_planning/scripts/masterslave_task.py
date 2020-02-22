from task import Task

class MasterSlaveTask(Task):

    def __init__(self, master, slave):
        super(MasterSlaveTask, self).__init__()
        self.master = master
        self.slave = slave

    def _on_task_run(self):
        if(self.master.finished):
            self.slave.finish()
