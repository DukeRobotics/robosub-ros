import rospy
import actionlib

from sonar.msg import sweepAction, sweepGoal

class sonar_client:

    def __init__(self):
        self._client = actionlib.SimpleActionClient('sonar_sweep', sweepAction)
        self._client.wait_for_server()

    def execute_sweep(self, sangle, eangle, range):
        goal = sweepGoal(start_angle=sangle, end_angle=eangle, distance_to_scan=range)
        self._client.send_goal(goal)
        self._client.wait_for_result()
        return self._client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publ`ish and subscribe over ROS.
        rospy.init_node('sonar_sweep_client_py')
        client = sonar_client()
        result = sonar_client.execute_sweep(150, 250, 5)
        print(result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")