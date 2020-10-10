from move_tasks import MoveToPoseGlobalTask
from combination_tasks import IndSimulTask, LeaderFollowerTask, ListTask
from task import Task

class GateTask(Task):
    def __init__(self):
        super(GateTask, self).__init__()

    def _on_task_start(self):
        self.threshold = 10
        self.begin_gate_search = IndSimulTask([DistanceToGateTask(self.threshold), IsThereAGateTask()])
        self.gate_search = LeaderFollowerTask(self.begin_gate_search, MoveToPoseGlobalTask(7, 0, 0, 0, 0, 0))
        self.after_gate_search_dummy = MoveToPoseGlobalTask(10, 0, 0, 0, 0, 0)
        self.go_to_gate = ListTask([self.gate_search, self.after_gate_search_dummy])

    def _on_task_run(self):


        """ 
        listtask1 = ListTask([
            LeaderFollowerTask(IsThereAGateTask, RotateTask(...)),
            select_smaller_box_of_seen_boxes_task,
            find_normal_vector_from_middle_of_smaller_box_task,
            move_to_and_align_with_normal_vector_task,
            LeaderFollowerTask(within_tolerance_of_normal_vector_task & DistanceToGateTask, ListTask([move_to_gate_task, calculate_normal_task, calculate_dist_task]))
            ])
        task1 = LeaderFollowerTask(DistanceToGateTask, listtask1)
        task2 = ListTask([CalculateNormalTask, AlignWithNormalTask, MoveTask])

        ListTask of listtask1, task1, task2
        """

        self.go_to_gate.run()



class DistanceToGateTask(Task):
    def __init__(self, threshold):
        super(DistanceToGateTask, self).__init__()
        self.threshold = threshold

    def _on_task_start(self):
        self.distance_counter_test = 300

    def _on_task_run(self):
        self.distance_counter_test -= 1
        print(self.distance_counter_test)
        if self.distance_counter_test < self.threshold:
            self.finish()


class IsThereAGateTask(Task):
    def __init__(self):
        super(IsThereAGateTask, self).__init__()

    def _on_task_run(self):
        self.finish()
        # if data_from_cv:
        #     self.finish()
