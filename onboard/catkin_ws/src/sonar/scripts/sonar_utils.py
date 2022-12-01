from geometry_msgs.msg import PoseStamped

def transform_pose(listener, base_frame, target_frame, pose):
    pose_stamped = PoseStamped()
    pose_stamped.pose = pose
    pose_stamped.header.frame_id = base_frame

    return listener.transformPose(target_frame, pose_stamped).pose
