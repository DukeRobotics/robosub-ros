import rospy
import numpy as np
import actionlib
from custom_msgs.msg import sweepAction, sweepGoal


SONAR_CENTER_GRADIANS = 200
RADIANS_PER_GRADIAN = np.pi / 200
GRADIANS_PER_DEGREE = 400 / 360


class SonarClient:
    def __init__(self):
        rospy.init_node('sonar_sweep_client_py')
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


def degrees_to_centered_gradians(angle_degrees):
    """ Converts degrees centered at 0 to gradians centered at 200

    Args:
        angle_degrees (float): Angle in degrees where 0 is forward

    Returns:
        int: Angle in gradians where 200 (Sonar.SONAR_CENTER_GRADIANS) is forward
    """

    angle_gradians = int(angle_degrees * GRADIANS_PER_DEGREE)
    angle_gradians_centered = angle_gradians + SONAR_CENTER_GRADIANS
    return angle_gradians_centered


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('sonar_sweep_client')
        client = SonarClient()
        result = client.sweep_at_center_angle(15, 2, 5)
        print(result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
