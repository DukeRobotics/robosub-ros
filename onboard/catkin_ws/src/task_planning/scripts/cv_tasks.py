import smach

# Says where to spin to center a given CV object in the frame
class SpinDirectionTask(smach.State):
    def __init__(self, name, tolerance, cv):
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