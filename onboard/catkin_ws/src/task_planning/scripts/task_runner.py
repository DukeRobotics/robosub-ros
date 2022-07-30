#!/usr/bin/env python3

import rospy
import smach.StateMachine
from gate_task import create_simple_gate_task_sm, create_simplest_gate_task_sm


class TaskRunner:
    RATE = 30  # Hz

    def __init__(self):
        rospy.init_node("task_planning")
        # SET THESE TO CURRENT WORKING STATE OF ROBOT
        self.YAW_WORKING = True
        self.CV_GATE_WORKING = True
        self.CV_BUOY_WORKING = True
        self.ACOUSTICS_WORKING = False

    def start(self):
        sm_top = smach.StateMachine(outcomes=['task_runner_succeeded', 'task_runner_failed'])

        with sm_top:
            # Determine gate task to run based on state of CV
            if self.CV_GATE_WORKING:
                sm_gate = create_simple_gate_task_sm(1)
            else:
                sm_gate = create_simplest_gate_task_sm(1)
            
            # Determine the next task to run based on state of CV and acoustics
            if self.CV_BUOY_WORKING:
                # If the bouy is working, move to buoy task
                next_step = 'BUOY_TASK'
            elif self.ACOUSTICS_WORKING:
                # If the acoustics are working, do octagon with acoustics
                next_step = 'ACOUSTICS_TASK'
            else:
                # If neither is working, do octagon without acoustics
                next_step = 'EYEBALL_OCTAGON_TASK'
            
            # Add the gate task selected earlier to the top state machine
            smach.StateMachine.add('GATE_TASK', sm_gate,
                                    transitions={
                                        'succeeded': next_step,
                                        'failed': 'task_runner_failed'
                                    })
            
            # If the buoy is working, add the buoy task
            if self.CV_BUOY_WORKING:
                sm_buoy = create_buoy_task_sm()
                if self.ACOUSTICS_WORKING:
                    next_step = 'ACOUSTICS_TASK'
                else:
                    next_step = 'task_runner_succeeded'
                smach.StateMachine.add('BUOY_TASK', sm_buoy,
                                        transitions={
                                            'succeeded': next_step,
                                            'failed': 'task_runner_failed'
                                        })
            
            # If the acoustics are working, add the acoustics task
            if self.ACOUSTICS_WORKING:
                sm_acoustics = create_acoustics_task_sm()
                smach.StateMachine.add('ACOUSTICS_TASK', sm_acoustics,
                                        transitions={
                                            'succeeded': 'task_runner_succeeded',
                                            'failed': 'task_runner_failed'
                                        })
            
            # If neither are working, just eyeball to octagon and then end
            elif self.CV_BUOY_WORKING == False:
                smach.StateMachine.add('EYEBALL_OCTAGON_TASK', create_eyeball_octagon_task_sm(),
                                        transitions={
                                            'succeeded': 'task_runner_succeeded',
                                            'failed': 'task_runner_failed'
                                        })


def main():
    try:
        TaskRunner().start()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
