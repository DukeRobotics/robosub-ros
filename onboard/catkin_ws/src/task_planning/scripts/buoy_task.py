from interface.controls import ControlsInterface
from interface.cv import CVInterface
import smach
import cv_tasks
from move_tasks import AllocateVelocityLocalTask, HoldPositionTask, MoveToUserDataPoseLocalTask, MoveToPoseLocalTask

class FunctionWrapper():
    def __init__(self, func, obj):
        self.func = func
        self.obj = obj

    def execute(self):
        ret = self.obj.execute()
        go = self.func()
        if not go:
            return 'func-done'
        return ret

class BuoyTask(smach.StateMachine):
    CENTER_TOLERANCE = 0.05
    ROTATE_SPEED = 1
    bonk_num = 0
    targets = ['bootleggerbuoy', 'bootleggerbuoy2']
    
    def upd_bonk(self):
        self.bonk_num += 1
        return self.bonk_num > 1
        

    def __init__(self, listener, controls: ControlsInterface, cv: CVInterface):
        super().__init__(outcomes=['done']) #maybe do calls here - also TODO add move backwards at the end of the func

        with self:
            for i in range(len(self.targets)):
                smach.StateMachine.add(f'CHOOSE_ROTATE_DIR_{i}',
                                    cv_tasks.SpinDirectionTask(self.targets[self.bonk_num], self.CENTER_TOLERANCE, cv),
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
                smach.StateMachine.add(f'BUOY_VALID_{i}', cv_tasks.ObjectCoordsValidTask(self.targets[self.bonk_num], cv),
                                    transitions={
                                            'valid': f'BUOY_POSE_{i}',
                                            'invalid': f'BUOY_VALID_{i}'
                                        })
                smach.StateMachine.add(f'BUOY_POSE_{i}', cv_tasks.ObjectPoseTask(self.targets[self.bonk_num], cv),
                                    transitions={
                                            'done': f'MOVE_TO_BUOY_{i}'
                                        }) # it's not letting me press enter what do I do omg this is so bad oh no this is so so bad
                smach.StateMachine.add(f'MOVE_TO_BUOY_{i}', MoveToUserDataPoseLocalTask(controls, listener),
                                    transitions={
                                            'continue': f'MOVE_BACK_{i}'
                                        })
                smach.StateMachine.add(f'MOVE_BACK_{i}', MoveToPoseLocalTask(-2, 0, 0, 0, 0, 0, controls, listener),
                                        transitions={
                                            'continue': f'MOVE_BACK_{i}',
                                            'done': f'CHOOSE_ROTATE_DIR_{i+1}' if i < len(self.targets) - 1 else 'done'
                                        })
                
