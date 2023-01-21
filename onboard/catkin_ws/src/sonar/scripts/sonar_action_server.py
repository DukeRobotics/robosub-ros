import roslib
import rospy
import actionlib
from sonar import Sonar
from custom_msgs.msg import sweepAction, sweepResult
roslib.load_manifest('sonar')


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

    _result = sweepResult()
    NODE_NAME = "sonar_server"
    ACTION_NAME = "sonar_sweep"

    def __init__(self):
        """ Creates action server with the sweepAction

        Callback: Runs the execute method

        """
        # 5m range
        self._sonar = Sonar()

        rospy.init_node(self.NODE_NAME)
        self._server = actionlib.SimpleActionServer(self.ACTION_NAME, sweepAction, self.execute, auto_start=False)
        self._server.start()
        rospy.spin()

    def execute(self, goal):
        self._sonar.set_new_range(goal.distance_of_scan)
        max_tuple = self._sonar.sweep_biggest_byte(goal.start_angle, goal.end_angle)

        # Sends everything as a result to the client
        self._result.angle_found = max_tuple[2]
        self._result.distance_to_sample = self._sonar.get_distance_of_sample(max_tuple[0])
        self._server.set_succeeded(self._result)


if __name__ == '__main__':
    SonarServer()
