from onboard.catkin_ws.src.task_planning.scripts.interface.controls import ControlsInterface
from onboard.catkin_ws.src.task_planning.scripts.interface.cv import CVInterface
import smach
import cv_tasks
from move_tasks import AllocateVelocityLocalTask, HoldPositionTask, MoveToUserDataPoseLocalTask


class BuoyTask(smach.StateMachine):
    CENTER_TOLERANCE = 0.05
    ROTATE_SPEED = 1

    def __init__(self, listener, controls: ControlsInterface, cv: CVInterface):
        super().__init__(outcomes=['done'])

        with self:
            smach.StateMachine.add('CHOOSE_ROTATE_DIR',
                                   cv_tasks.SpinDirectionTask('bootleggerbuoy', self.CENTER_TOLERANCE, cv),
                                   transitions={
                                        'left': 'ROTATE_LEFT',
                                        'right': 'ROTATE_RIGHT',
                                        'center': 'BUOY_CENTERED'
                                    })

            smach.StateMachine.add('ROTATE_LEFT',
                                   AllocateVelocityLocalTask(0, 0, 0, 0, 0, self.ROTATE_SPEED, controls),
                                   transitions={
                                        'done': 'CHOOSE_ROTATE_DIR'
                                    })
            smach.StateMachine.add('ROTATE_RIGHT',
                                   AllocateVelocityLocalTask(0, 0, 0, 0, 0, -self.ROTATE_SPEED, controls),
                                   transitions={
                                        'done': 'CHOOSE_ROTATE_DIR'
                                    })

            # We are now facing the buoy
            smach.StateMachine.add('BUOY_CENTERED', HoldPositionTask(controls),
                                   transitions={
                                        'done': 'BUOY_VALID'
                                    })
            smach.StateMachine.add('BUOY_VALID', cv_tasks.ObjectCoordsValidTask('bootleggerbuoy', cv),
                                   transitions={
                                        'valid': 'BUOY_POSE',
                                        'invalid': 'BUOY_VALID'
                                    })
            smach.StateMachine.add('BUOY_POSE', cv_tasks.ObjectPoseTask('bootleggerbuoy', cv),
                                   transitions={
                                        'done': 'MOVE_TO_BUOY'
                                    })
            smach.StateMachine.add('MOVE_TO_BUOY', MoveToUserDataPoseLocalTask(controls, listener),
                                   transitions={
                                        'continue': 'done'
                                    })
            # TODO make sure we aren't too close to see it
