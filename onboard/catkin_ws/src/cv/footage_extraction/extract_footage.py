from cv_bridge import CvBridge
import rosbag
import cv2
import image_tools
import os


# Extracts footage from rosbag files
class ROSBagFootageExtractor:

    def view_rosbag_info(self, footage_path):
        """
        Call rosbag info on the rosbag file. This provides a human-readable summary of the file contents, including
        what topic was recorded.

        :param footage_path: Path to rosbag file.
        :return: Prints summary of the file contents.
        """
        os.system("rosbag info " + footage_path)
        print()

    def extract_frames(self, footage_path, topic_name, output_dir, step_size=10):
        """
        Extract frames from the rosbag file and saves them into self.output_dir

        :param footage_path: Path to rosbag file.
        :param topic_name: Name of the topic where the image feed is published. Can find by calling view_rosbag_info
        on the rosbag file.
        :param output_dir: Path for directory to save the output. Will create a subdirectory within output_dir for each
        file that is extracted.
        :param step_size: Will extract a frame every step_size number of frames.
        :return: Will
        """

        file_basename = os.path.basename(footage_path)
        file_name_no_extension = os.path.splitext(file_basename)[0]

        bag = rosbag.Bag(footage_path)

        frame_count = 0
        num_frames_saved = 0
        image_tools = image_tools.ImageTools()

        save_dir = os.path.join(output_dir, file_name_no_extension)
        if not os.path.exists(save_dir):
            os.mkdir(save_dir)

        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            if topic == topic_name:
                if frame_count % step_size == 0:
                    cv_img = image_tools.convert_ros_msg_to_cv2(msg)

                    file_save_path = os.path.join(save_dir, f"{file_name_no_extension}_{frame_count}.jpg")
                    success = cv2.imwrite(file_save_path, cv_img)

                    print(f"File path: {file_save_path}, success: {success}")

                    if success:
                        num_frames_saved += 1

                frame_count += 1

        print(f"Read {frame_count} frames")
        print(f"Saved {num_frames_saved} frames to {output_dir + file_name}")

        bag.close()

if __name__ == '__main__':

    FOOTAGE_PATH = '/root/dev/robosub-ros/footage_extraction/footage/test.bag'
    OUTPUT_DIR = '/root/dev/robosub-ros/footage_extraction/footage/'
    TOPIC_NAME = "/camera/front/stream_raw"

    footage_extractor = ROSBagFootageExtractor()

    footage_extractor.view_rosbag_info(FOOTAGE_PATH)

    footage_extractor.extract_frames(FOOTAGE_PATH, TOPIC_NAME, OUTPUT_DIR)
