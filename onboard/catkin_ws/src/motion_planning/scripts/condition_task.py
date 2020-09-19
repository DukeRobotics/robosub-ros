from task import Task

class ConditionTaskBase(Task):
    def __init__(self, condition):
        self.condition = condition

    def _on_task_run(self):
        if self.condition:
            self.finish()
