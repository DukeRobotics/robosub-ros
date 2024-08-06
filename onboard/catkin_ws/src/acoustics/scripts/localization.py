import rospy
from std_msgs.msg import Int8
import random

def localization_pub():

    # -1 means we are under the octagon
    # 0-3 inclusive are the 4 quadrants relative to the robot heading

    pub = rospy.Publisher('/acoustics/localization', Int8, queue_size=10)
    rospy.init_node('localization_pub', anonymous=True)

    while not rospy.is_shutdown():
        localization = random.randint(-1, 3)
        rospy.loginfo(localization)
        pub.publish(localization)

if __name__ == '__main__':
    try:
        localization_pub()
    except rospy.ROSInterruptException:
        pass

