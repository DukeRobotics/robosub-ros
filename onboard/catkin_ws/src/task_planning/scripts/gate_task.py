from move_tasks import MoveToPoseGlobalTask, AllocatePowerTask, MoveToPoseLocalTask
from combination_tasks import IndSimulTask, DepSimulTask, LeaderFollowerTask, ListTask
from task import Task
from geometry_msgs.msg import Point


class GateTask(Task):
    DIST_THRESH = 5  # distance to get before blindly move through gate

    def __init__(self, power=0.2):
        super(GateTask, self).__init__()
        self.power = power

    def _on_task_start(self):
        self.gate_search_condition = IndSimulTask([DistanceToGateTask(self.DIST_THRESH), ListTask([IsThereAGateTask()], -1)]
        self.rotate_to_gate = LeaderFollowerTask(IsThereAGateTask(), AllocatePowerTask(0, 0, 0, 0, 0, self.power))
        self.move_along_x_y = MoveToGateTask()
        self.gate_path_condition = OnGatePathTask()  # finishes if fall off path or lose camera data
        self.move_forward = LeaderFollowerTask(self.gate_path_condition, AllocatePowerTask(self.power, 0, 0, 0, 0, 0))
        self.gate_sequence = ListTask([self.rotate_to_gate,
                                       self.move_along_x_y,
                                       self.move_forward], 100)
        self.move_through_gate = AllocatePowerTask(self.power, 0, 0, 0, 0, 0)
        self.do_gate_magic = LeaderFollowerTask(self.gate_search_condition,
                                             ListTask[self.gate_sequence, self.move_through_gate])

    def _on_task_run(self):
        self.do_gate_magic.run()


class MoveToGateTask(MoveToPoseGlobalTask):  # this will be local
    def __init__(self):
        super(MoveToGateTask, self).__init__(0, 0, 0, 0, 0, 0)

    def _on_task_start(self):
        self.gate_vector = self.gate_data
        self.desired_pose.position = self.gate_vector
        #  super(MoveToGateTask, self)._on_task_start()  # transforms pose


class DistanceToGateTask(Task):
    def __init__(self, threshold):
        super(DistanceToGateTask, self).__init__()
        self.threshold = threshold

    def _on_task_start(self):
        self.distance_counter_test = 300

    def _on_task_run(self):
        # self.distance_counter_test -= 1
        # print(self.distance_counter_test)
        if self.distance_counter_test < self.threshold:
            self.finish()


class IsThereAGateTask(Task):  # finishes if cannot see a gate
    def __init__(self):
        super(IsThereAGateTask, self).__init__()

    def _on_task_run(self):
        self.finish()
        #  if (self.gate_data.x != None) & (self.gate_data.y != None) & (self.gate_data.z != None):   
        #      self.finish()


class OnGatePathTask(Task):
    def __init__(self):
        super(OnGatePathTask, self).__init__()

    def _on_task_run(self):
        if self.gate_data.x == None:  # if gate data is bad finish
            self.finish()
        desired_pose = self.state.pose.pose
        desired_pose.position.x = self.gate_data.x
        desired_pose.position.y = self.gate_data.y
        if !task_utils.at_pose(self.state.pose.pose, desired_pose):
            self.finish()


# write some function that checks if gate data is bad