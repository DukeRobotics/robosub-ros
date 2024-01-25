from sonar import Sonar
from sonar_image_processing import scan_and_build_sonar_image
import rospy
import sys
from sonar_utils import degrees_to_centered_gradians

if __name__ == "__main__":
    rospy.init_node("local_pool_test")
    sonar = Sonar(5)
    start = int(degrees_to_centered_gradians(int(sys.argv[1])))
    end = int(degrees_to_centered_gradians(int(sys.argv[2])))
    scan_and_build_sonar_image(sonar, False,
                               jpeg_save_path="Sonar_Image_pool_test.jpeg",
                               start_angle=start,
                               end_angle=end)
