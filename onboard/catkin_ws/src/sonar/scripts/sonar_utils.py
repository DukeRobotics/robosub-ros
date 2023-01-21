from geometry_msgs.msg import PoseStamped

def transform_pose(listener, pose):
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
    pose_stamped.header.frame_id = "sonar_link"

    return listener.transformPose("base_link", pose_stamped).pose
