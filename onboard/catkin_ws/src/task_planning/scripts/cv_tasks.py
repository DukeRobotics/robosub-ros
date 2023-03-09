from onboard.catkin_ws.src.task_planning.scripts.interface.cv import CVInterface
import smach


# Says where to spin to center a given CV object in the frame
class SpinDirectionTask(smach.State):
    '''Says where to spin to center a given CV object in the frame'''
    def __init__(self, name: str, tolerance: float, cv: CVInterface):
        '''
        Says where to spin to center a given CV object in the frame
        
        Parameters
        name : str
            Name of the CV object to center
        tolerance : float
            How close to the center the object must be to be considered centered
        cv : CVInterface
            CV object to get data from
        '''
        super(SpinDirectionTask, self).__init__(outcomes=["left", "right", "center"])
        self.name = name
        self.tolerance = tolerance
        self.cv = cv

    def execute(self, _):
        cv_data = self.cv.get_data(self.name)
        if cv_data is None or cv_data.xmin > 0.5 + self.tolerance:
            return "left"
        if cv_data.xmax < 0.5 - self.tolerance:
            return "right"
        return "center"


# TODO maybe delete in favor of always using ObjectCoordsTask
class ObjectCoordsValidTask(smach.State):
    def __init__(self, name, cv):
        super().__init__(outcomes=["valid", "invalid"])
        self.name = name
        self.cv = cv

    def execute(self, _):
        cv_data = self.cv.get_data(self.name)
        if cv_data is None or not cv_data.sonar:
            return "invalid"
        return "valid"


class ObjectCoordsTask(smach.State):
    def __init__(self, name, cv):
        super().__init__(outcomes=["valid", "invalid"], output_keys=["coords"])
        self.name = name
        self.cv = cv

    def execute(self, userdata):
        cv_data = self.cv.get_data(self.name)
        if cv_data is None or not cv_data.sonar:
            return "invalid"
        userdata.coords = cv_data.coords
        return "valid"
