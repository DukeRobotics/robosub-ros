import roslib
roslib.load_manifest('sonar')
import rospy
import actionlib
from sonar import Sonar

from custom_msgs.msg import sweepAction, sweepResult
""" The  following messages are created by the .action file
    sweepAction.msg
    sweepActionGoal.msg 
    sweepActionResult.msg
    sweepActionFeedback.msg

    sweepGoal.msg                   #the parameters passed from the client
        int32 start_angle
        int32 end_angle
        int32 distance_of_scan
    sweepResult.msg                 #whats returned from to the client
        int32 angle_found
        int32 distance_to_sample
    sweepFeedback.msg               #sent as continuous feedback to the robot (not implimented)
        int32 current_angle
"""

class SonarServer:

    DEFAULT_RANGE = 5 #m
    _result = sweepResult()

    def __init__(self):
        """ Creates action server with the sweepAction

        Callback: Runs the execute method

        """
        rospy.init_node('sonar_sweep')
        self._server = actionlib.SimpleActionServer(rospy.get_name(), sweepAction, self.execute, auto_start=False)
        self._server.start()

        #5m range
        self._sonar = Sonar(self.DEFAULT_RANGE)
        
    def execute(self, goal):
        self._sonar.set_new_range(goal.distance_of_scan)
        max_tuple = self._sonar.sweep_biggest_byte(goal.start_angle, goal.end_angle)

        self._result.angle_found = max_tuple[2]
        self._result.distance_to_sample = self._sonar.get_distance_of_sample(max_tuple[0])
        self._server.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('sonar_server')
    server = SonarServer()
    while not rospy.is_shutdown():
        rospy.spin()
    #TODO fix while not rospy.is_shutdown() method
