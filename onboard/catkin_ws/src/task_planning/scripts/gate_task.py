from move_tasks import MoveToPoseGlobalTask, AllocateVelocityLocalTask, MoveToPoseLocalTask
from combination_tasks import IndSimulTask, DepSimulTask, LeaderFollowerTask, ListTask
from task import Task
import task_utils
import numpy as np
from geometry_msgs.msg import Point

class GateTask(Task):
    SIDE_THRESHOLD = 0.1  # means gate post is within 1 tenth of the side of the frame
    CENTERED_THRESHOLD = 0.1  # means gate will be considered centered if within 1 tenth of the center of the frame

    def __init__(self, velocity=0.2):
        super(GateTask, self).__init__()
        self.velocity = velocity

    def _on_task_start(self):
        self.gate_search_condition = NearGateTask(self.SIDE_THRESHOLD)  # finishes when can see gate and close to gate
        self.rotate_condition = GateCenteredTask(self.CENTERED_THRESHOLD)  # finishes when gate is centered
        self.rotate = LeaderFollowerTask(self.rotate_condition, AllocateVelocityLocalTask(0, 0, 0, 0, 0, self.velocity))
        self.advance_condition = GateCenteredTask(self.CENTERED_THRESHOLD, return_on_center=False)  # finishes when/if gate is not centered
        self.advance = LeaderFollowerTask(self.center_condition, AllocateVelocityLocalTask(self.velocity, 0, 0, 0, 0, 0))
        self.gate_magic = ListTask([
            LeaderFollowerTask(self.gate_search_condition, ListTask([self.rotate, self.advance], -1)),
            MoveToPoseLocalTask(3, 0, 0, 0, 0, 0)])

    def _on_task_run(self):
        self.gate_magic.run()
        if self.gate_magic.finished:
            self.finish()


"""

        self.gate_search_condition = IndSimulTask(
            [DistanceToGateTask(self.DIST_THRESH), ListTask([IsThereAGateTask()], -1)])

        self.rotate_to_gate = LeaderFollowerTask(IsThereAGateTask(), AllocateVelocityTask(0, 0, 0, 0, 0, self.velocity))
        self.move_to_gate = LeaderFollowerTask(IsThereAGateTask(finish_with_data=False), MoveToGateTask())
        # self.gate_path_condition = OnGatePathTask()  # finishes if fall off path or lose camera data
        # self.move_forward = LeaderFollowerTask(self.gate_path_condition, AllocatePowerTask(self.power, 0, 0, 0, 0, 0))
        self.gate_sequence = ListTask([self.rotate_to_gate, self.move_to_gate], 100)
        self.move_through_gate = AllocateVelocityTask(self.velocity, 0, 0, 0, 0, 0)
        self.do_gate_magic = LeaderFollowerTask(self.gate_search_condition,
                                                ListTask[self.gate_sequence, self.move_through_gate])
"""


    def scrutinize_gate(self, gate_data, gate_tick_data):
        """Finds the distance from the gate to each of the four edges of the frame

        Parameters:
        gate_data (custom_msgs/CVObject): cv data for the gate
        gate_tick_data (custom_msgs/CVObject): cv data for the gate tick

        Returns:
        dict: left - distance from left edge of gate to frame edge (from 0 to 1)
              right - distance from right edge of gate to frame edge (from 0 to 1)
              top - distance from top edge of gate to frame edge (from 0 to 1)
              bottom - distance from bottom edge of gate to frame edge (from 0 to 1)
              offset_h - difference between distances on the right and left sides (from 0 to 1)
              offset_v - difference between distances on the top and bottom sides (from 0 to 1)
        """
        if gate_data.label == 'none':
            return None
        res = {}
        res["left"] = gate_data.xmin
        res["right"] = 1 - gate_data.xmax
        res["top"] = gate_data.ymin
        res["bottom"] = 1 - gate_data.ymax

        # Adjust the target area if the gate tick is detected
        if gate_tick_data.label != 'none' and gate_tick_data.score > 0.5:
            # If the tick is closer to the left side
            if abs(gate_tick_data.xmin - gate_data.xmin) < abs(gate_tick_data.xmax - gate_data.xmax):
                res["right"] = 1 - gate_tick_data.xmax
            else:
                res["left"] = gate_tick_data.xmin
            
            res["bottom"] = 1 - gate_tick_data.ymax

        res["offset_h"] = res["right"] - res["left"]
        res["offset_v"] = res["bottom"] - res["left"]

        return res



class NearGateTask(Task):
    def __init__(self, threshold):
        super(NearGateTask, self).__init__()
        self.threshold = threshold

    def _on_task_run(self):
        gate_info = scrutinize_gate(self.gate_data, self.gate_tick_data)
        if gate_info:
            if (gate_info["left"] > self.threshold) and (gate_info["right"] > self.threshold):
                self.finish()


class GateCenteredTask(Task):
    def __init__(self, threshold, return_on_center=True):
        super(GateCenteredTask, self).__init__()
        self.threshold = threshold
        self.return_on_center = return_on_center

    def _on_task_run(self):
        gate_info = scrutinize_gate(self.gate_data, self.gate_tick_data)
        if gate_info:
            if self.return_on_center:
                if gate_info["offset_h"] < threshold:
                    self.finish()
            else:
                if gate_info["offset_h"] > threshold:
                    self.finish()

"""
class IsThereAGateTask(Task):  # finishes if there is gate data that is NOT None
    def __init__(self):
        super(IsThereAGateTask, self).__init__()

    def _on_task_run(self):
        gate_info = scrutinize_gate(self.gate_data, self.gate_tick_data)
        if gate_info:
            self.finish()








class MoveToGateTask(Task):  # this can be generalized to dynamic point, relies on gate data already accounting for single bad frames
    def __init__(self):
        super(MoveToGateTask, self).__init__()

    def _on_task_run(self):
        self.global_pose = task_utils.transform('base_link', 'odom', self.gate_data)
        self.publish_desired_pose_global(self.global_pose)
        at_desired_pose_vel = task_utils.stopped_at_pose(self.state.pose.pose, self.transformed_pose, self.state.twist.twist)
        if at_desired_pose_vel:
            self.finish()


class DistanceToGateTask(Task):
    def __init__(self, threshold):
        super(DistanceToGateTask, self).__init__()
        self.threshold = threshold

    def _on_task_start(self):
        self.distance_to_gate = np.linalg.norm(self.gate_data.x, self.gate_data.y, self.gate_data.z)

    def _on_task_run(self):
        if self.distance_to_gate < self.threshold:
            self.finish()


class OnGatePathTask(Task):
    def __init__(self):
        super(OnGatePathTask, self).__init__()

    def _on_task_run(self):
        if self.gate_data.x is None:  # if gate data is bad finish
            self.finish()
        desired_pose = self.state.pose.pose
        desired_pose.position.x = self.gate_data.x
        desired_pose.position.y = self.gate_data.y
        if not task_utils.at_pose(self.state.pose.pose, desired_pose):
            self.finish()
"""