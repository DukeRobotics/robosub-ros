from sonar import Sonar
from sonar_image_processing import *
import rospy

# COMMAND TO SETUP SERVER:
# python -m http.server 
# Search 192.161.1.1:8000 in google :)

if __name__ == "__main__":
    rospy.init_node("local_pool_test")
    sonar = Sonar(5) #need to test different # of samples - would also have to make 
    scan_and_build_sonar_image(sonar, False, jpeg_save_path="Sonar_Image.jpg", start_angle=197, end_angle=203)
