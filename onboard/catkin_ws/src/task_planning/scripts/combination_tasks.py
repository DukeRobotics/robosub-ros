
from task import Task
import copy


class ListTask(Task):
    """Run a list of tasks sequentially"""

    def __init__(self, tasks):
        super(ListTask, self).__init__()

        self.tasks = tasks

    def _on_task_start(self):
        self.curr_index = 0

    def _on_task_run(self):
        if self.curr_index == len(self.tasks):
            self.finish()

        elif self.tasks[self.curr_index].finished:
            self.curr_index += 1

        else:
            self.tasks[self.curr_index].run()


class RepeatTask(Task):
    """Runs a task num_reps times, runs indefinitely if num_reps=-1"""

    def __init__(self, task, num_reps):
        super(RepeatTask, self).__init__()

        self.task = task
        self.num_reps = num_reps

    def _on_task_start(self):
        self.curr_rep = -1

    def _on_task_run(self):
        if self.curr_rep == -1 or self.copy.finished:
            self.copy = copy.deepcopy(self.task)
            self.curr_rep += 1  # curr_rep will be 0 on first rep

        if self.num_reps == self.curr_rep:
            self.finish()

        self.copy.run()


class IndSimulTask(Task):
    """Run a list of tasks simulataneously, exits when all tasks finished"""

    def __init__(self, tasks):
        super(IndSimulTask, self).__init__()

        self.tasks = tasks

    def _on_task_run(self):
        all_finished = True
        for task in self.tasks:
            task.run()
            if not task.finished:
                all_finished = False

        if all_finished:
            self.finish()


class DepSimulTask(Task):
    """Run a list of tasks simulataneously, exits when any task finished"""

    def __init__(self, tasks):
        super(DepSimulTask, self).__init__()

        self.tasks = tasks

    def _on_task_run(self):
        any_finished = False
        for task in self.tasks:
            task.run()
            if task.finished:
                any_finished = True

        if any_finished:
            for task in self.tasks:
                task.finish()
            self.finish()


class LeaderFollowerTask(Task):

    def __init__(self, leader, follower):
        super(LeaderFollowerTask, self).__init__()
        self.leader = leader
        self.follower = follower

    def _on_task_run(self):
        if self.leader.finished:
            self.follower.finish()
            self.finish()

        self.leader.run()
        self.follower.run()


class IfElseTask(Task):

    def __init__(self, condition, task_one, task_two):
        super(IfElseTask, self).__init__()
        self.condition = condition
        self.task_one = task_one
        self.task_two = task_two

    def _on_task_start(self):
        if self.condition:
            self.task_running = self.task_one
        else:
            self.task_running = self.task_two
        self.task_running.run()

    def _on_task_run(self):
        if self.task_running.finished:
            self.finish()
