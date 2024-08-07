import rospy
from std_msgs.srv import SetBool

class SaleaeEnable:
    SERVICE_TOPIC = '/acoustics/toggle_saleae'

    def __init__(self):
        rospy.init_node('saleae_enable')
        rospy.Service(self.SERVICE_TOPIC, SetBool, self.toggle_saleae)

    def call_saleae_enable_script(self):
        # Call the run.sh script in this package
        pass

    def toggle_saleae(self, req):
        if req.enable:
            rospy.loginfo('Enabling Saleae')

        else:
            rospy.loginfo('Disabling Saleae')
        return True

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = SaleaeEnable()
    node.run()
