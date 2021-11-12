#!/usr/bin/env python

import smach
import rospy
from task import Task
from move_tasks import MoveToPoseLocalTask, AllocateVelocityLocalTask
from tf import TransformListener
from time import sleep


SIDE_THRESHOLD = 0.1  # means gate post is within 1 tenth of the side of the frame
CENTERED_THRESHOLD = 0.1  # means gate will be considered centered if within 1 tenth of the center of the frame

def main():
    rospy.init_node('gate_task')
    
    sm = create_gate_task_sm()

    # Execute SMACH plan
    outcome = sm.execute()

def create_gate_task_sm(velocity=0.2):
    sm = smach.StateMachine(outcomes=['gate_task_succeeded', 'gate_task_failed'])
    sleep(2)
    listener = TransformListener()
    with sm:
        smach.StateMachine.add('NEAR_GATE', NearGateTask(SIDE_THRESHOLD),
                               transitions={
                                   'true': 'MOVE_THROUGH_GATE',
                                   'false': 'HORIZONTAL_ALIGNMENT',
                                   'spin': 'NEAR_GATE'})

        smach.StateMachine.add('MOVE_THROUGH_GATE', MoveToPoseLocalTask(3, 0, 0, 0, 0, 0, listener),
                               transitions={
                                   'done': 'gate_task_succeeded'})

        smach.StateMachine.add('HORIZONTAL_ALIGNMENT', GateHorizontalAlignmentTask(CENTERED_THRESHOLD),
                               transitions={
                                   'left': 'ROTATE_LEFT',
                                   'right': 'ROTATE_RIGHT',
                                   'center': 'VERTICAL_ALIGNMENT',
                                   'spin': 'HORIZONTAL_ALIGNMENT'})

        smach.StateMachine.add('ROTATE_LEFT', AllocateVelocityLocalTask(0, 0, 0, 0, 0, velocity),
                               transitions={'done': 'HORIZONTAL_ALIGNMENT'})

        smach.StateMachine.add('ROTATE_RIGHT', AllocateVelocityLocalTask(0, 0, 0, 0, 0, -velocity),
                               transitions={'done': 'HORIZONTAL_ALIGNMENT'})

        smach.StateMachine.add('VERTICAL_ALIGNMENT', GateVerticalAlignmentTask(CENTERED_THRESHOLD),
                               transitions={
                                   'top': 'ASCEND',
                                   'bottom': 'DESCEND',
                                   'center': 'ADVANCE',
                                   'spin': 'VERTICAL_ALIGNMENT'})

        smach.StateMachine.add('ASCEND', AllocateVelocityLocalTask(0, 0, velocity, 0, 0, 0),
                               transitions={'done': 'VERTICAL_ALIGNMENT'})

        smach.StateMachine.add('DESCEND', AllocateVelocityLocalTask(0, 0, -velocity, 0, 0, 0),
                               transitions={'done': 'VERTICAL_ALIGNMENT'})

        smach.StateMachine.add('ADVANCE', AllocateVelocityLocalTask(velocity, 0, 0, 0, 0, 0),
                               transitions={'done': 'NEAR_GATE'})

    return sm


class GateHorizontalAlignmentTask(Task):
    def __init__(self, threshold):
        super(GateHorizontalAlignmentTask, self).__init__(["center","right","left","spin"])
        self.threshold = threshold

    def run(self, userdata):
        gate_info = _scrutinize_gate(self.cv_data['gate'], self.cv_data['gate_tick'])
        if gate_info:
            if abs(gate_info["offset_h"]) < self.threshold:
                return "center"
            if gate_info["offset_h"] < 0:
                return "right"
            return "left"
        return "spin"


class GateVerticalAlignmentTask(Task):
    def __init__(self, threshold):
        super(GateVerticalAlignmentTask, self).__init__(["center","top","bottom","spin"])
        self.threshold = threshold

    def run(self, userdata):
        gate_info = _scrutinize_gate(self.cv_data['gate'], self.cv_data['gate_tick'])
        if gate_info:
            if abs(gate_info["offset_v"]) < self.threshold:
                return "center"
            if gate_info["offset_v"] < 0:
                return "top"
            return "bottom"
        return "spin"


class NearGateTask(Task):
    def __init__(self, threshold):
        super(NearGateTask, self).__init__(["true","false","spin"])
        self.threshold = threshold

    def run(self, userdata):
        print(self.cv_data)
        gate_info = _scrutinize_gate(self.cv_data['gate'], self.cv_data['gate_tick'])
        if gate_info:
            if (gate_info["left"] > self.threshold) and (gate_info["right"] > self.threshold):
                return "true"
            else:
                return "false"
        return "spin"


def _scrutinize_gate(gate_data, gate_tick_data):
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
    if not(gate_data) or not(gate_tick_data) or gate_data.label == 'none':
        return None

    res = {
        "left": gate_data.xmin,
        "right": 1 - gate_data.xmaxg,
        "top": gate_data.ymin,
        "bottom": 1 - gate_data.ymax
    }

    # Adjust the target area if the gate tick is detected
    if gate_tick_data.label != 'none' and gate_tick_data.score > 0.5:
        # If the tick is closer to the left side
        if abs(gate_tick_data.xmin - gate_data.xmin) < abs(gate_tick_data.xmax - gate_data.xmax):
            res["right"] = 1 - gate_tick_data.xmax
        else:
            res["left"] = gate_tick_data.xmin
        
        res["bottom"] = 1 - gate_tick_data.ymax

    res["offset_h"] = res["right"] - res["left"]
    res["offset_v"] = res["bottom"] - res["top"]

    return res

if __name__ == '__main__':
    main()