from combination_tasks import ListTask
from move_tasks import MoveToPoseGlobalTask
from task import Task
from gate_task import GateTask
from style_task import StyleTask
# from prequal_tasks import PreQualGlobalTask


class CompetitionTask(Task):
    """
    High level competition level task, contains a list of tasks for each competition task.
    """

    def __init__(self, *args, **kwargs):
        super(CompetitionTask, self).__init__(*args, **kwargs)

        # self.gate = GateTask()
        # self.list_task = ListTask([MoveToPoseGlobalTask(-7, 0, 0, 0, 0, 0)])
        self.style = StyleTask("z", 0.2)

    def _on_task_run(self):
        self.style.run()

        if self.style.finished:
            self.finish()
