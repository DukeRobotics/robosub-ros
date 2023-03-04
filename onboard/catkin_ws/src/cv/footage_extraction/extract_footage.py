import rosbag
import cv2
import image_tools
import os
import yaml
import sys


FOOTAGE_EXTRACTION_DIR = '/root/dev/robosub-ros/onboard/catkin_ws/src/cv/footage_extraction'
FOOTAGE_DIR = os.path.join(FOOTAGE_EXTRACTION_DIR, 'footage')
EXTRACTED_FOOTAGE_DIR = os.path.join(FOOTAGE_EXTRACTION_DIR, 'extracted_footage')
CONFIG_FILE = os.path.join(FOOTAGE_EXTRACTION_DIR, 'footage_extraction_config.yaml')


# Extracts footage from rosbag files
class FootageExtractor:

    def view_rosbag_info(self, footage_path):
        """
        Call rosbag info on the rosbag file. This provides a human-readable summary of the file contents, including
        what topic was recorded.

        :param footage_path: Path to rosbag file.
        :return: Prints summary of the file contents.
        """
        os.system("rosbag info " + footage_path)
        print()

    def extract_frames_from_bag(self, footage_path, topic_name, output_dir, step_size=10):
        """
        Extract frames from the rosbag file and saves them into self.output_dir

        :param footage_path: Path to rosbag file.
        :param topic_name: Name of the topic where the image feed is published. Can find by calling view_rosbag_info
        on the rosbag file.
        :param output_dir: Path for directory to save the output. Will create a subdirectory within output_dir for each
        file that is extracted.
        :param step_size: Will extract a frame every step_size number of frames.
        """

        file_basename = os.path.basename(footage_path).replace(".", "_")

        bag = rosbag.Bag(footage_path)

        num_frames = bag.get_message_count('/'+topic_name)
        frame_count = 0
        num_frames_saved = 0
        num_digits = len(str(num_frames))
        img_tools = image_tools.ImageTools()

        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            if topic == topic_name:
                if frame_count % step_size == 0:
                    current_frame_str = str(frame_count)
                    current_frame_str = current_frame_str.rjust(num_digits, '0')
                    cv_img = img_tools.convert_to_cv2(msg)

                    file_save_path = os.path.join(output_dir, f"{file_basename}_frame_{current_frame_str}.jpg")
                    success = cv2.imwrite(file_save_path, cv_img)

                    if success:
                        num_frames_saved += 1

                frame_count += 1

        bag.close()

    # Function to extract frames from a video file
    # For each frame in the video at videop_path, save the frame to output_dir
    # Only take a frame every step_size frames
    # Filenames should be output_dir/filename_frame_number.jpg (replace any '.' in filename with '_')
    def extract_frames_from_video(self, video_filename, video_path, output_dir, step_size=10):
        cam = cv2.VideoCapture(video_path)
        num_frames = int(cam.get(cv2.CAP_PROP_FRAME_COUNT))
        num_digits = len(str(num_frames))

        current_frame = 0
        ret, frame = cam.read()

        # while there is still video left, continue creating images
        while ret:
            if current_frame % step_size == 0:
                current_frame_str = str(current_frame)
                current_frame_str = current_frame_str.rjust(num_digits, '0')
                converted_video_filename = video_filename.replace('.', '_')
                name = os.path.join(output_dir, f"{converted_video_filename}_frame_{current_frame_str}.jpg")
                cv2.imwrite(name, frame)

            # iterate to the next frame in the video
            current_frame += 1
            ret, frame = cam.read()

        # Release all space and windows once done
        cam.release()
        cv2.destroyAllWindows()

    # Function to prepare a config file for the footage extractor
    # Input: directory and list of files in directory
    # Output: Save YAML file with the following format to directory/footage_extraction_config.yaml:
    # bag_filename_1:
    #   enabled: False
    #   image_topic_name_1: False
    #   image_topic_name_2: False
    #   ...
    # bag_filename_2:
    #   enabled: False
    #   image_topic_name_1: False
    #   image_topic_name_2: False
    #   ...
    # video_filename_1:
    #   enabled: False
    # video_filename_2:
    #   enabled: False
    # ...
    def create_config_file(self, directory):
        config_dict = {}
        files = os.listdir(directory)

        for file in files:
            converted_file_name = file.replace(".", "_")
            file_dict = {'enabled': False}

            if file.endswith("bag"):
                path = os.path.join(directory, file)
                bag = rosbag.Bag(path, "r")
                topics = bag.get_type_and_topic_info()[1].keys()
                for topic in topics:
                    file_dict[topic] = False

            config_dict[converted_file_name] = file_dict

        yaml_string = yaml.dump(config_dict)

        with open(CONFIG_FILE, "w") as yaml_file:
            yaml_file.write(yaml_string)

    # Function to read a config file (see above YAML) and extract footage accordingly
    # Call the appropriate functions to extract footage from the files specified in the config file
    # Only extract footage from files that are enabled in the config file
    # Only extract footage from topics that are enabled in the config file
    # For video files, extraction path should be directory/extracted_footage/filename,
    # but replace any '.' in filename with '_'
    # For example, for extracting pool_test_video.mp4, the output path
    # should be directory/extracted_footage/pool_test_video_mp4
    # For bag files, create a folder for the bag file, and within that,
    # create a folder for each topic, replacing any '/' with '_'
    # For example, for extracting bag_file.bag, the output path should be
    # directory/extracted_footage/bag_file_bag/image_topic_name
    # Make sure to create the directories first before extracting footage
    def extract_footage_with_config(self, directory, config_file):
        with open(config_file, 'r') as file:
            configs = yaml.safe_load(file)

        for file in configs:
            config_dict = configs[file]
            if config_dict['enabled']:

                # creating output directory
                output_dir = os.path.join(EXTRACTED_FOOTAGE_DIR, file)
                os.mkdir(output_dir)

                # extracting file_name
                file_name = file[::-1].replace('_', '.', 1)[::-1]
                footage_path = os.path.join(directory, file_name)

                if file_name.endswith('bag'):
                    for topic in config_dict:
                        if topic == 'enabled':  # The key 'enabled' is not a topic
                            continue

                        if config_dict[topic]:

                            # creating topic directory
                            converted_topic_name = topic.replace("/", "_")[1:]
                            topic_dir = os.path.join(output_dir, converted_topic_name)
                            os.mkdir(topic_dir)

                            self.extract_frames_from_bag(footage_path, topic, topic_dir)

                else:  # if file is video
                    self.extract_frames_from_video(file_name, footage_path, output_dir)


if __name__ == '__main__':

    # Using command line arguments, check if the --generate_config flag is set
    # If it is, call the create_config_file function
    # If it is not, ask the user to confirm they want to extract footage on the command line
    # If user confirms, call the extract_footage_with_config function

    footage_extractor = FootageExtractor()

    arg_list = sys.argv[1:]

    generate_config_flag = False
    for arg in arg_list:
        if arg.startswith('--generate_config'):
            generate_config_flag = True

    if generate_config_flag:
        footage_extractor.create_config_file(FOOTAGE_DIR)
    else:
        inp = input("Are you sure you want to extract footage? (Y/N): ")
        if inp.lower() == "y":
            footage_extractor.extract_footage_with_config(FOOTAGE_DIR, CONFIG_FILE)