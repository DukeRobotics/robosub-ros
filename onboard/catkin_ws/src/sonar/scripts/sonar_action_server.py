import roslib
roslib.load_manifest('sonar')
import rospy
import actionlib
from sonar import Sonar

from sonar.msg import sweepAction, sweepResult

class SonarServer:

    DEFAULT_RANGE = 5 #m
    _result = sweepResult()

    def __init__(self):
        self._server = actionlib.SimpleActionServer('sonar_sweep', sweepAction, self.execute, auto_start=False)
        self._server.start()

        #5m range
        self._sonar = Sonar(self.DEFAULT_RANGE)
        
    
    def execute(self, goal):
        self._sonar.set_new_range(goal.distance_of_scan)
        max_tuple = self._sonar.sweep_biggest_byte(goal.start_angle, goal.end_angle)
        self._result.angle_found = max_tuple[2]
        self._result.distance_to_sample = self._sonar.get_distance_of_sample(max_tuple[0])
        self.server.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('sonar_server')
    server = SonarServer()
    rospy.spin()
    #TODO impliment while not rospy.is_shutdown() method
