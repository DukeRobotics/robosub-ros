from task import Task
from combination_tasks import ListTask
from move_tasks import MoveToPoseGlobalTask
from move_tasks import MoveToPoseLocalTask
from move_tasks import HoldPositionTask

class PreQualGlobalTask(Task):
    """
    Task to complete prequalifying run by moving to global poses
    """

    POSE1 = [0, 0, 0, 0, 0, 0]
    POSE2 = [0, 0, 0, 0, 0, 0]
    POSE3 = [0, 0, 0, 0, 0, 0]
    POSE4 = [0, 0, 0, 0, 0, 0]
    POSE5 = [0, 0, 0, 0, 0, 0]

    def __init__(self, *args, **kwargs):
        super(PreQualGlobalTask, self).__init__(*args, **kwargs)

        self.x1, self.y1, self.z1, self.
        self.list_task = ListTask([ MoveToPoseGlobalTask(*POSE1), 
                                    MoveToPoseGlobalTask(*POSE2),
                                    MoveToPoseGlobalTask(*POSE3),
                                    MoveToPoseGlobalTask(*POSE4),
                                    MoveToPoseGlobalTask(*POSE5) ])

    def _on_task_run(self):
        self.list_task.run()

        if self.list_task.finished:
            self.finish()


class PreQualLocalTask(Task):
    """
    Task to complete prequalifying run by moving to local poses
    """

    POSE1 = [0, 0, 0, 0, 0, 0]
    POSE2 = [0, 0, 0, 0, 0, 0]
    POSE3 = [0, 0, 0, 0, 0, 0]
    POSE4 = [0, 0, 0, 0, 0, 0]
    POSE5 = [0, 0, 0, 0, 0, 0]

    def __init__(self, *args, **kwargs):
        super(PreQualLocalTask, self).__init__(*args, **kwargs)

        self.x1, self.y1, self.z1, self.
        self.list_task = ListTask([ MoveToPoseGlobalTask(*POSE1), 
                                    MoveToPoseGlobalTask(*POSE2),
                                    MoveToPoseGlobalTask(*POSE3),
                                    MoveToPoseGlobalTask(*POSE4),
                                    MoveToPoseGlobalTask(*POSE5) ])

    def _on_task_run(self):
        self.list_task.run()

        if self.list_task.finished:
            self.finish()