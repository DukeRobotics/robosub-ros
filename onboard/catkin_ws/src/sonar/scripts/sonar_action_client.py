import rospy
import actionlib
from custom_msgs.msg import sweepAction, sweepGoal


class SonarClient:
    def __init__(self):
        self._client = actionlib.SimpleActionClient('sonar_sweep', sweepAction)
        self._client.wait_for_server()

    def execute_sweep(self, sangle, eangle, range):
        goal = sweepGoal(start_angle=sangle, end_angle=eangle, distance_of_scan=range)
        self._client.send_goal(goal)
        self._client.wait_for_result()
        return self._client.get_result()

    #takes angle in degrees
    def sweep_at_center_angle(self, center, radius, range = 5):

        center = center + 180

        left = max(center - radius, 0)
        right = min(center+radius, 400)
        left = int((400/360) * left)
        right = int((400/360) * right)
        return self.execute_sweep(left, right, range)
if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('sonar_sweep_client_py')
        client = SonarClient()
        result = client.execute_sweep(150, 250, 5)
        print(result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
