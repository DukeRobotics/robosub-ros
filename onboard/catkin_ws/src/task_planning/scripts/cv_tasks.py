from interface.cv import CVInterface
import smach
import rospy
import task_utils


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


class ObjectCoordsTask(smach.State):
    def __init__(self, name, cv: CVInterface):
        super().__init__(outcomes=["valid", "invalid"], output_keys=["point"])
        self.name = name
        self.cv = cv

    def execute(self, userdata):
        cv_data = self.cv.get_data(self.name)
        if cv_data is None or not cv_data.sonar:
            return "invalid"
        userdata.point = cv_data.coords
        return "valid"


# Might need a refactor
class ObjectVisibleTask(smach.State):
    def __init__(self, image_name, timeout=0):
        super(ObjectVisibleTask, self).__init__(["undetected", "detected"],
                                                input_keys=['image_name'],
                                                output_keys=['image_name'])
        self.image_name = image_name
        self.timeout = timeout  # in seconds

    def execute(self, userdata):
        cycles_per_second = 10
        rate = rospy.Rate(cycles_per_second)
        total = 0
        while total <= self.timeout * cycles_per_second:
            if task_utils.object_vector(self.cv_data[self.image_name]) is not None:
                return "detected"
            total += 1
            rate.sleep()
        return "undetected"
