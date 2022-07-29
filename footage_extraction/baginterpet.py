from cv_bridge import CvBridge
import rosbag
import cv2
import image_tools
import os

FILE_NAME = "test_depthai_camera_stream-7-28-2022"
FILE_EXTENSION = ".bag"
FOLDER_PATH = '/root/dev/robosub-ros/footage_extraction/footage/' # with / at end
FOOTAGE_PATH = FOLDER_PATH + FILE_NAME + FILE_EXTENSION
FRAMES_PATH = FOLDER_PATH + FILE_NAME + '/'
STEP_SIZE = 10
TOPIC_NAME = "/camera/front/stream_raw"

print("Do you want to view rosbag info? (y/n): ", end="")
view_rosbag_info = input()

if view_rosbag_info == "y":
    # terminal command for rosbag info
    os.system("rosbag info " + FOOTAGE_PATH)
    print()

print("Do you want to continue with extracting frames from the rosbag? (y/n): ", end="")
extract_rosbag_frames = input()
if extract_rosbag_frames == "y":
    bag = rosbag.Bag(FOOTAGE_PATH)
    bridge = CvBridge()
    count = 0
    image_tools = image_tools.ImageTools()
    for topic, msg, t in bag.read_messages(topics=[TOPIC_NAME]):
        if topic == TOPIC_NAME:
            if count % STEP_SIZE == 0:
                print(msg)
                cv_img = image_tools.convert_ros_compressed_to_cv2(msg)
                print(cv_img)
                # cv2.imwrite(FRAMES_PATH + FILE_NAME + "%d.jpg" % count, cv_img)
            count += 1

    print("Read " + str(count) + " frames")

    # Close the bagfile
    bag.close()