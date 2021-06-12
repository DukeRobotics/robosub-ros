from combination_tasks import ListTask
from move_tasks import MoveToPoseGlobalTask
from move_tasks import MoveToPoseLocalTask
from task import Task


class PreQualGlobalTask(Task):
    """
    Task to complete prequalifying run by moving to global poses
    """

    POSE1 = [0, 0, 0, 0, 0, 0]
    POSE2 = [0, 0, 0, 0, 0, 0]
    POSE3 = [0, 0, 0, 0, 0, 0]
    POSE4 = [0, 0, 0, 0, 0, 0]
    POSE5 = [0, 0, 0, 0, 0, 0]

    def __init__(self):
        super(PreQualGlobalTask, self).__init__()

        self.list_task = ListTask([MoveToPoseGlobalTask(*self.POSE1),
                                   MoveToPoseGlobalTask(*self.POSE2),
                                   MoveToPoseGlobalTask(*self.POSE3),
                                   MoveToPoseGlobalTask(*self.POSE4),
                                   MoveToPoseGlobalTask(*self.POSE5)])

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

    def __init__(self):
        super(PreQualLocalTask, self).__init__()

        self.list_task = ListTask([MoveToPoseLocalTask(*self.POSE1),
                                   MoveToPoseLocalTask(*self.POSE2),
                                   MoveToPoseLocalTask(*self.POSE3),
                                   MoveToPoseLocalTask(*self.POSE4),
                                   MoveToPoseLocalTask(*self.POSE5)])

    def _on_task_run(self):
        self.list_task.run()

        if self.list_task.finished:
            self.finish()
