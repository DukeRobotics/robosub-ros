from interface.controls import ControlsInterface
from interface.cv import CVInterface
import smach
import cv_tasks, task_utils
from move_tasks import AllocateVelocityLocalTask, HoldPositionTask, MoveToUserDataPoseLocalTask, MoveToPoseLocalTask


class BuoyTask(smach.StateMachine):
    CENTER_TOLERANCE = 0.05
    ROTATE_SPEED = 1
    TOO_CLOSE_TO_BUOY = 0.5
    targets = ['buoy_abydos_taurus', 'buoy_abydos_sepenscaput']

    def __init__(self, listener, controls: ControlsInterface, cv: CVInterface):
        super().__init__(outcomes=['done']) #maybe do calls here - also TODO add move backwards at the end of the func

        with self:
            for i in range(len(self.targets)):
                smach.StateMachine.add(f'CHOOSE_ROTATE_DIR_{i}',
                                    cv_tasks.SpinDirectionTask(self.targets[i], self.CENTER_TOLERANCE, cv),
                                    transitions={
                                            'left': f'ROTATE_LEFT_{i}',
                                            'right': f'ROTATE_RIGHT_{i}',
                                            'center': f'BUOY_CENTERED_{i}'
                                        })

                smach.StateMachine.add(f'ROTATE_LEFT_{i}',
                                    AllocateVelocityLocalTask(0, 0, 0, 0, 0, self.ROTATE_SPEED, controls),
                                    transitions={
                                            'done': f'CHOOSE_ROTATE_DIR_{i}'
                                        })
                smach.StateMachine.add(f'ROTATE_RIGHT_{i}',
                                    AllocateVelocityLocalTask(0, 0, 0, 0, 0, -self.ROTATE_SPEED, controls),
                                    transitions={
                                            'done': f'CHOOSE_ROTATE_DIR_{i}'
                                        })

                # We are now facing the buoy
                smach.StateMachine.add(f'BUOY_CENTERED_{i}', HoldPositionTask(controls),
                                    transitions={
                                            'done': f'BUOY_VALID_{i}'
                                        })
                smach.StateMachine.add(f'BUOY_VALID_{i}', cv_tasks.ObjectCoordsTask(self.targets[i], cv),
                                    transitions={
                                            'valid': f'BUOY_POSE_{i}',
                                            'invalid': f'BUOY_VALID_{i}'
                                        })
                smach.StateMachine.add(f'BUOY_POSE_{i}', task_utils.PointToPoseTask(controls),
                                    transitions={
                                            'done': f'BUOY_TOO_CLOSE_{i}'
                                        })
                smach.StateMachine.add(f'BUOY_TOO_CLOSE_{i}', task_utils.LambdaTask(self.check_too_close,
                                                                                    ['close', 'far'], input_keys=['pose']),
                                    transitions={
                                            'close': f'BONK_BUOY_{i}',
                                            'far': f'APPROACH_BUOY_{i}'
                                        })
                smach.StateMachine.add(f'APPROACH_BUOY_{i}', MoveToUserDataPoseLocalTask(controls, listener),
                                    transitions={
                                            'continue': f'BUOY_VALID_{i}',
                                            'done': f'MOVE_BACK_{i}' # We should switch to BONK_BUOY_ before this
                                        })
                smach.StateMachine.add(f'BONK_BUOY_{i}', MoveToUserDataPoseLocalTask(controls, listener),
                                    transitions={
                                            'continue': f'BONK_BUOY_{i}',
                                            'done': f'MOVE_BACK_{i}'
                                        })
                smach.StateMachine.add(f'MOVE_BACK_{i}', MoveToPoseLocalTask(-2, 0, 0, 0, 0, 0, controls, listener),
                                        transitions={
                                            'continue': f'MOVE_BACK_{i}',
                                            'done': f'CHOOSE_ROTATE_DIR_{i+1}' if i < len(self.targets) - 1 else 'done'
                                        })

    def check_too_close(self, userdata):
        if userdata.pose.position.x**2 + userdata.pose.position.y**2 + userdata.pose.position.z**2 > self.TOO_CLOSE_TO_BUOY:
            return 'far'
        return 'close'
                
