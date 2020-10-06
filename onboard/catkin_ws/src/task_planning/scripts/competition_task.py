from combination_tasks import ListTask
from move_tasks import MoveToPoseGlobalTask
from task import Task
from gate_task import GateTask
# from prequal_tasks import PreQualGlobalTask


class CompetitionTask(Task):
    """
    High level competition level task, contains a list of tasks for each competition task.
    """

    def __init__(self, *args, **kwargs):
        super(CompetitionTask, self).__init__(*args, **kwargs)

        #self.gate = GateTask()
        self.list_task = ListTask([MoveToPoseGlobalTask(-7, 0, 0, 0, 0, 0)])

    def _on_task_run(self):
        self.list_task.run()
        #self.gate.run()

        if self.list_task.finished:
            self.finish()
