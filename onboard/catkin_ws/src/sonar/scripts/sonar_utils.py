from geometry_msgs.msg import PoseStamped
import numpy as np


SONAR_CENTER_GRADIANS = 200
RADIANS_PER_GRADIAN = np.pi / 200
GRADIANS_PER_DEGREE = 400 / 360


def transform_pose(listener, base_frame, target_frame, pose):
    """ Transform pose from base reference frame to target reference frame frame

    Args:
        listener (TransformListener): Transform listener
        base_frame (str): Name of the base frame
        target_frame (str): Name of the target frame
        pose (Pose): Pose in base reference frame to transform into target frame

    Returns:
        Pose: Pose in target reference frame
    """
    
    pose_stamped = PoseStamped()
    pose_stamped.pose = pose
    pose_stamped.header.frame_id = base_frame

    return listener.transformPose(target_frame, pose_stamped).pose


def centered_gradians_to_radians(self, angle_gradians):
    """ Converts gradians centered at 200 to radians centered at 0

    Args:
        angle_gradians (float): Angle in gradians where 200 (Sonar.SONAR_CENTER_GRADIANS) is forward

    Returns:
        float: Angle in radians
    """
    angle_radians = (angle_gradians - self.SONAR_CENTER_GRADIANS) * RADIANS_PER_GRADIAN
    return angle_radians


def degrees_to_centered_gradians(self, angle_degrees):
    """ Converts degrees centered at 0 to gradians centered at 200

    Args:
        angle_degrees (float): Angle in degrees where 0 is forward

    Returns:
        int: Angle in gradians where 200 (Sonar.SONAR_CENTER_GRADIANS) is forward
    """
    angle_gradians = int(angle_degrees * GRADIANS_PER_DEGREE)
    angle_gradians_centered = angle_gradians + self.SONAR_CENTER_GRADIANS
    return angle_gradians_centered
