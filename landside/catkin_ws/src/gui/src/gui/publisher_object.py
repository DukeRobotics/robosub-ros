import rospy
from python_qt_binding.QtCore import QObject, QReadWriteLock 

class PublisherObject(QObject):

    def __init__(self, publisher, start_msg):
        super(PublisherObject, self).__init__()
        self.publisher = publisher
        self.msg = start_msg
        self.lock = QReadWriteLock()
        self.running = True

    def run(self):
        rate = rospy.Rate(15)
        while self.running:
            self.lock.lockForRead()
            self.publisher.publish(self.msg)
            self.lock.unlock()
            rate.sleep()
    
    def update_msg(self, msg):
        self.lock.lockForWrite()
        self.msg = msg
        self.lock.unlock()

    def stop(self):
        self.running = False
