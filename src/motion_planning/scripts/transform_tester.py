from task_utils import transform
from geometry_msgs.msg import Pose, Odometry
import rospy



def transform(origin, destination, odometry=None, pose=None):
    """Transforms Odometry input from origin frame to destination frame
    
    Arguments:
    origin: the starting frame
    destination: the frame to trasform to
    odometry: the odometry message to transform

    Returns:
    The transformed odometry message
    """
 
   if(odometry != None):
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        trans = tfBuffer.lookup_transform(origin, destination, rospy.Time(0))
        #TODO: transform odometry    
   elif(pose != None):
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        trans = tfBuffer.lookup_transform(origin, destination, rospy.Time(0))
	transformed = tf2_geometry_msgs.do_transform_pose(pose, trans)


pose = Pose()
pose.position.x = 2
pose.position.y = 4
pose.position.z = 7 

transformedPose = transform('origin','desitnation', pose)

print(transformedPose.position.x)
print(transformedPose.position.y)
print(transformedPose.position.z)
