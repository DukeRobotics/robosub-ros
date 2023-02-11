from sonar import Sonar
from sonar_image_processing import *
import rospy

if __name__ == "__main__":
    rospy.init_node("local_pool_test")
    sonar = Sonar(5) #need to test different # of samples - would also have to make 
    build_sonar_image(sonar, False, jpeg_save_path="/sampleData/testImgage", start_angle=197, end_angle=203)
