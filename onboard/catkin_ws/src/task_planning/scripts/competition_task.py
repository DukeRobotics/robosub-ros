from combination_tasks import ListTask
from move_tasks import MoveToPoseGlobalTask
from task import Task
#from gate_task import GateTask
from style_task import StyleTask
from gate_task import MoveToGateTask, GateTask
# from prequal_tasks import PreQualGlobalTask


class CompetitionTask(Task):
    """
    High level competition level task, contains a list of tasks for each competition task.
    """

    def __init__(self, *args, **kwargs):
        super(CompetitionTask, self).__init__(*args, **kwargs)

        
    def _on_task_start(self):
        # self.gate = GateTask()
        # self.list_task = ListTask([MoveToPoseGlobalTask(-7, 0, 0, 0, 0, 0)])
        self.test = GateTask()

    def _on_task_run(self):
        self.test.run()
        if self.test.finished:
            self.finish()
