from cv_bridge import CvBridge
import rosbag
import cv2
import image_tools
import os

FILE_NAME = "gate_and_buoy_footage_3"
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
    frame_count = 0
    num_frames_saved = 0
    image_tools = image_tools.ImageTools()

    save_dir = os.path.join(FOLDER_PATH, FILE_NAME)
    if not os.path.exists(save_dir):
        os.mkdir(save_dir)

    for topic, msg, t in bag.read_messages(topics=[TOPIC_NAME]):
        if topic == TOPIC_NAME:
            if frame_count % STEP_SIZE == 0:
                cv_img = image_tools.convert_ros_msg_to_cv2(msg)
                file_save_path = os.path.join(save_dir, f"{FILE_NAME}_{frame_count}.jpg")
                success = cv2.imwrite(file_save_path, cv_img)
                print(f"File path: {file_save_path}, success: {success}")
                if success:
                    num_frames_saved += 1
            frame_count += 1

    print(f"Read {frame_count} frames")
    print(f"Saved {num_frames_saved} frames to {FRAMES_PATH + FILE_NAME}")

    # Close the bagfile
    bag.close()