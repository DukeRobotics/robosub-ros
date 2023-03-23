import rospy
from custom_msgs.msg import CVObject
from move_tasks import MoveToPoseGlobalTask


class TestMoveSubscriber:
    def __init__(self):
        # gate, gate_side, gate_tick, gate_top, start_gate, buoy_bootlegger, buoy_gman
        path = 'gate'
        self.sub = rospy.Subscriber(f"/move_base/result/{path}", CVObject, self.callback)

    def callback(self, obj: CVObject):
        MoveToPoseGlobalTask(*self.imlazy(obj.pose.position, 'x', 'y', 'z'), 0, 0, 0).execute()

    def imlazy(obj, *args):
        '''Returns a list of attributes of an object.'''
        items = []
        for arg in args:
            items.append(getattr(obj, arg))
        return obj
