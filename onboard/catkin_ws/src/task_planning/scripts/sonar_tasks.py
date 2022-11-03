from task import Task
from geometry_msgs.msg import Vector3


class SweepTask(Task):
    def __init__(self):
        super(SweepTask, self).__init__(["done"])

    def run(self, userdata):
        # TODO perform sweep
        return "done"


class GateSweepTask(Task):
    """Finds the gate and returns the position of each leg in userdata
    """

    def __init__(self):
        super(GateSweepTask, self).__init__(["done"], output_keys=["left", "right"])

    def run(self, userdata):
        # TODO perform sweep
        userdata.left = Vector3(0, 0, 0)
        userdata.right = Vector3(0, 0, 0)
        return "done"