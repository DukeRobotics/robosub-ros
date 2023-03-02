from sonar import Sonar
from sonar_image_processing import scan_and_build_sonar_image
import rospy
import sys

# COMMAND TO SETUP SERVER:
# python -m http.server
# Search 192.168.1.1:8000 in google :)

if __name__ == "__main__":
    rospy.init_node("local_pool_test")
    sonar = Sonar(5)
    scan_and_build_sonar_image(sonar, False,
                               jpeg_save_path="Sonar_Image.jpeg",
                               start_angle=int(sys.argv[1]),
                               end_angle=int(sys.argv[2]))
