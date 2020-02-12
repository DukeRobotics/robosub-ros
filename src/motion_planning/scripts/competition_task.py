from task import Task
from combination_tasks import ListTask
from move_tasks import MoveToPoseLocalTask

class CompetitionTask(Task):
    """
    High level competition level task, contains a list of tasks for each competition task.
    """

    def __init__(self, *args, **kwargs):
        super(CompetitionTask, self).__init__(*args, **kwargs)

        self.list_task = ListTask([ MoveToPoseLocalTask(2, 0, 0, 0, 0, 0) ])

    def _on_task_run(self):
        self.list_task.run()

        if self.list_task.finished:
            self.finish()