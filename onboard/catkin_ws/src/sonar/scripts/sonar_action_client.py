import rospy
import actionlib
from custom_msgs.msg import sweepAction, sweepGoal
from sonar_utils import degrees_to_centered_gradians


class SonarClient:
    def __init__(self):
        self._client = actionlib.SimpleActionClient('sonar_sweep', sweepAction)
        self._client.wait_for_server()

    def execute_sweep(self, sangle, eangle, scan_distance):
        goal = sweepGoal(start_angle=sangle, end_angle=eangle, distance_of_scan=scan_distance)
        self._client.send_goal(goal)
        self._client.wait_for_result()
        return self._client.get_result()

    # takes angle in degrees
    def sweep_at_center_angle(self, center_degrees, breadth_degrees, scan_distance=5):

        center_gradians = degrees_to_centered_gradians(center_degrees)
        breadth_gradians = degrees_to_centered_gradians(breadth_degrees)

        left = max(center_gradians - breadth_gradians, 0)
        right = min(center_gradians + breadth_gradians, 400)

        return self.execute_sweep(left, right, scan_distance)


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
