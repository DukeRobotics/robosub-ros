from task import Task
# from gate_task import GateTask
# from style_task import StyleTask
# from gate_task import MoveToGateTask, GateTask
# from prequal_tasks import PreQualGlobalTask
from move_tasks import AllocateVelocityGlobalTask


class CompetitionTask(Task):
    """
    High level competition level task, contains a list of tasks for each competition task.
    """

    def __init__(self, *args, **kwargs):
        super(CompetitionTask, self).__init__(*args, **kwargs)

    def _on_task_start(self):
        # self.gate = GateTask()
        # self.list_task = ListTask([MoveToPoseGlobalTask(-7, 0, 0, 0, 0, 0)])
        # self.test = GateTask()
        # self.test_vel_local = AllocateVelocityLocalTask(0.2, 0, 0, 0, 0, 0)
        self.test_vel_global = AllocateVelocityGlobalTask(0.2, 0, 0, 0, 0, 0)
        # self.test_style = StyleTask("z", .2)

    def _on_task_run(self):
        self.test_vel_global.run()
        if self.test_vel_global.finished:
            self.finish()

# this checking condition from parent task is likely not how we want to actually
# do it, just for this pool test

        # self.test_vel_local.run()
        # desired_pose = Pose()
        # desired_pose.position.x = 1
        # desired_pose.position.y = 0
        # desired_pose.position.z = 0
        # if task_utils.at_pose(self.state.pose.pose, desired_pose):
        #     self.test_vel_local.finish()
        #     self.finish()

        # self.test_vel_global.run()
        # desired_pose = Pose()
        # desired_pose.position.x = 1
        # desired_pose.position.y = 0
        # desired_pose.position.z = 0
        # if task_utils.at_pose(self.state.pose.pose, desired_pose):
        #     self.test_vel_global.finish()
        #     self.finish()
