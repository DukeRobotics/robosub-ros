class GateTask(Task):
	def __init__(self):
		super(GateTask, self).__init__(*args, **kwargs)


	def _on_task_run(self):
	    listtask1 = ListTask([
	        MasterSlaveTask(IsThereAGateTask, RotateTask(...)),
	        select_smaller_box_of_seen_boxes_task,
	        find_normal_vector_from_middle_of_smaller_box_task,
	        move_to_and_align_with_normal_vector_task,
	        MasterSlaveTask(within_tolerance_of_normal_vector_task & DistanceToGateTask, ListTask([move_to_gate_task, calculate_normal_task, calculate_dist_task]))
	        ])
	    task1 = MasterSlaveTask(DistanceToGateTask, listtask1)
	    task2 = ListTask([CalculateNormalTask, AlignWithNormalTask, MoveTask])

	    ListTask of listtask1, task1, task2

class DistanceToGateTask(Task):
    def __init__(self, threshold):
        super...

    def _on_task_run(self):
        if(distance < threshold):
            self.finish()

class IsThereAGateTask(Task):
    def __init__(self):
        super...

    def _on_task_run(self):
        if data_from_cv:
            self.finish()
