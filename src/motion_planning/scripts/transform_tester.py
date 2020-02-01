<<<<<<< HEAD
from geometry_msgs.msg import Pose
import rospy
import tf2_ros

rospy.init_node('testing')
def transform(origin, destination, odometry=None, pose=None):
=======
#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped
import rospy
import tf2_ros
import tf2_geometry_msgs

rospy.init_node('testing')

def transform(origin, destination, this_pose=None):
>>>>>>> e99b556cb5afa4f99b468a06738e2673f131c60c
    """Transforms Odometry input from origin frame to destination frame

    Arguments:
    origin: the starting frame
    destination: the frame to trasform to
    odometry: the odometry message to transform

    Returns:
    The transformed odometry message
    """

    if(this_pose != None):
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
<<<<<<< HEAD
        trans = tfBuffer.lookup_transform(destination, origin, rospy.Time(0))
        #TODO: transform odometry
    elif(pose != None):
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        trans = tfBuffer.lookup_transform(origin, destination, rospy.Time(0))
        transformed = tf2_geometry_msgs.do_transform_pose(pose, trans)
=======
        trans = tfBuffer.lookup_transform(destination, origin, rospy.Time(), rospy.Duration(0.5))
        transformed_pose = tf2_geometry_msgs.do_transform_pose(this_pose, trans)

    return transformed_pose
>>>>>>> e99b556cb5afa4f99b468a06738e2673f131c60c


pose_stamped = PoseStamped()
pose_stamped.pose.position.x = 2
pose_stamped.pose.position.y = 4
pose_stamped.pose.position.z = 7

<<<<<<< HEAD
transformedPose = transform('origin','destination', pose)
=======
transformed_pose = transform('origin1', 'destination1', pose_stamped)
>>>>>>> e99b556cb5afa4f99b468a06738e2673f131c60c

print(transformed_pose.pose.position.x)
print(transformed_pose.pose.position.y)
print(transformed_pose.pose.position.z)
