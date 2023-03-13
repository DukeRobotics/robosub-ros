import rosbag
import cv2
import image_tools
import os
import yaml
import sys
import shutil
from datetime import datetime
import roboflow


FOOTAGE_EXTRACTION_DIR = '/root/dev/robosub-ros/onboard/catkin_ws/src/cv/footage_extraction'
FOOTAGE_DIR = os.path.join(FOOTAGE_EXTRACTION_DIR, 'footage')
EXTRACTED_FOOTAGE_DIR = os.path.join(FOOTAGE_EXTRACTION_DIR, 'extracted_footage')
EXTRACTED_FILES_DIR = os.path.join(FOOTAGE_EXTRACTION_DIR, 'extracted_files')
UPLOADED_FOOTAGE = os.path.join(FOOTAGE_EXTRACTION_DIR, 'uploaded_fottage')
FOOTAGE_EXTRACTION_CONFIG_FILE = os.path.join(FOOTAGE_EXTRACTION_DIR, 'footage_extraction_config.yaml')
ROBOFLOW_UPLOAD_CONFIG_FILE = os.path.join(FOOTAGE_EXTRACTION_DIR, 'roboflow_upload_config.yaml')
ROBOFLOW_PROJECT_CONFIG_FILE = os.path.join(FOOTAGE_EXTRACTION_DIR, 'roboflow_project_config.yaml')


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

    def extract_frames_from_bag(self, topic_name, topic_use_name, footage_path, output_dir, step_size=10):
        """
        Extract frames from the rosbag file and saves them into output_dir

        :param footage_path: Path to rosbag file.
        :param topic_name: Name of the topic where the image feed is published. Can find by calling view_rosbag_info
        on the rosbag file.
        :param topic_use_name: Topic name that is used in the extracted frame names. Can set different from topic_name.
        :param output_dir: Path for directory to save the output. Will create a subdirectory within output_dir for each
        file that is extracted.
        :param step_size: Will extract a frame every step_size number of frames.
        """

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
                    file_save_path = os.path.join(output_dir, f"{topic_use_name}_frame_{current_frame_str}.jpg")
                    success = cv2.imwrite(file_save_path, cv_img)

                    if success:
                        num_frames_saved += 1

                frame_count += 1

        bag.close()

        # print filename and the number of frames read, extracted, and skipped
        print(f"Read {num_frames} frames, extracted {num_frames_saved} frames,",
              f"skipped {num_frames-num_frames_saved} frames from file {os.path.basename(footage_path)}")

    def extract_frames_from_video(self, use_name, video_path, output_dir, step_size=10):
        """
        Extract frames from the video file and saves them into output_dir

        :param use_name: Name that is used in the extracted frame names. Can be set different from video name.
        :param video_path: Path to video file.
        :param topic_name: Name of the topic where the image feed is published. Can find by calling view_rosbag_info
        on the rosbag file.
        :param output_dir: Path for directory to save the output. Will create a subdirectory within output_dir for each
        file that is extracted.
        :param step_size: Will extract a frame every step_size number of frames.
        """

        cam = cv2.VideoCapture(video_path)
        num_frames = int(cam.get(cv2.CAP_PROP_FRAME_COUNT))
        num_digits = len(str(num_frames))
        num_frames_saved = 0

        current_frame = 0
        ret, frame = cam.read()

        # while there is still video left, continue creating images
        while ret:
            if current_frame % step_size == 0:
                current_frame_str = str(current_frame)
                current_frame_str = current_frame_str.rjust(num_digits, '0')
                name = os.path.join(output_dir, f"{use_name}_frame_{current_frame_str}.jpg")
                cv2.imwrite(name, frame)
                num_frames_saved += 1

            # iterate to the next frame in the video
            current_frame += 1
            ret, frame = cam.read()

        # release all space and windows once done
        cam.release()
        cv2.destroyAllWindows()

        # print filename and the number of frames read, extracted, and skipped
        print(f"Read {num_frames} frames, extracted {num_frames_saved} frames,",
              f"skipped {num_frames-num_frames_saved} frames from file {os.path.basename(video_path)}")

    # TODO: Develop a function that uploads all images in given directory to the rf_project under batch_name
    # The documentation for the roboflow package is available here: https://docs.roboflow.com/python
    # You may also find the GitHub repo for the package useful: https://github.com/roboflow/roboflow-python
    def upload_images_to_roboflow(self, rf_project, directory, batch_name, uploaded_dir):
        """
        Upload all images in directory to the rf_project in Roboflow under batch_name

        :param rf_project: Name of Roboflow project
        :param directory: Path for directory containing all the images to be uploaded
        :param batch_name: Name of batch under rf_project
        """
        images = os.listdir(directory)
        status = {
            'successful': [],
            'non_successful': []
        }

        for image in images:
            image_path = os.path.join(directory, image)

            # if directory contains topic subdirectories inside, call function recursively on subdirectory
            if os.path.isdir(image_path):
                uploaded_topic_dir = os.path.join(uploaded_dir, image)
                os.mkdir(uploaded_topic_dir)
                return self.upload_images_to_roboflow(rf_project, image_path, batch_name, uploaded_topic_dir)
            else:
                # success = self.upload_images_to_roboflow_with_success(rf_project, image_path, batch_name)
                rf_project.upload(image_path, batch_name=batch_name)

                # if success:
                #     status['successful'].append(image)
                #     shutil.move(image_path, uploaded_dir)
                # else:
                #     status['unsuccessful'].append(image)

        # return status

    def upload_images_to_roboflow_with_success(self, rf_project, image_path, batch_name):
        success = False

        if not os.path.isfile(image_path):
            print(f"ERROR: The provided image path {image_path} is not a valid path.")

        elif not rf_project.check_valid_image(image_path):
            print(f"ERROR: The image at {image_path} is not a supported file format.",
                  "Only PNG and JPEG files are supported.")

        else:
            response = rf_project._Project__image_upload(image_path, batch_name)
            if response.json().get["duplicate"]:
                success = True
                print("Duplicate image not uploaded: " + image_path)

            else:
                success = response.json()["success"]

        if not success:
            print(f"ERROR: Server rejected image: {response.json()}. Image at {image_path} failed to upload!")

        return success

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

    # TODO: Modify config file structure to the following (added use_name and step_size keys).
    #
    # The value of use_name key specifies the name of the filenames and topics to use
    # when creating the extract directory. This is especially useful when the filenames
    # and topic names are long and we don't want long directory and filenames or if we want to extract
    # frames from the same file twice.
    #
    # The value of step_size key specifies the number of frames to skip between each frame that is extracted
    # for each topic in bag files and each video file.
    #
    # bag_filename_1:
    #   enabled: False
    #   use_name: bag_filename_1
    #   image_topic_name_1:
    #       enabled: False
    #       use_name: image_topic_name_1
    #       step_size: 10
    #   image_topic_name_2:
    #       enabled: False
    #       use_name: image_topic_name_2
    #       step_size: 10
    #   ...
    # bag_filename_2:
    #   enabled: False
    #   use_name: bag_filename_2
    #   image_topic_name_1:
    #       enabled: False
    #       use_name: image_topic_name_1
    #       step_size: 10
    #   image_topic_name_2:
    #       enabled: False
    #       use_name: image_topic_name_2
    #       step_size: 10
    #   ...
    # video_filename_1:
    #   enabled: False
    #   use_name: video_filename_1
    #   step_size: 10
    # video_filename_2:
    #   enabled: False
    #   use_name: video_filename_2
    #   step_size: 10
    # ...
    # TODO: Modify name of this function to create_footage_extraction_config_file
    def create_footage_extraction_config_file(self, directory, enabled=False, step_size=10):
        config_dict = {}
        files = os.listdir(directory)

        for file in files:
            if file.startswith("."):  # Skip over hidden files
                continue

            converted_file_name = file.replace(".", "_")
            file_dict = {'enabled': enabled, 'use_name': converted_file_name}

            if file.endswith("bag"):
                path = os.path.join(directory, file)
                bag = rosbag.Bag(path, "r")
                topics = bag.get_type_and_topic_info()[1].keys()
                for topic in topics:
                    use_name = topic.replace("/", "_")[1:]
                    topic_dict = {
                        'enabled': enabled,
                        'use_name': use_name,
                        'step_size': step_size
                    }

                    file_dict[topic] = topic_dict
            else:
                file_dict['step_size'] = step_size

            config_dict[converted_file_name] = file_dict

        yaml_string = yaml.dump(config_dict)

        with open(FOOTAGE_EXTRACTION_CONFIG_FILE, "w") as yaml_file:
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

    # TODO: For all files whose footage was extracted, move them to EXTRACTED_FILES_DIR
    def extract_footage_with_config(self, directory, config_file):
        with open(config_file, 'r') as file:
            configs = yaml.safe_load(file)

        for file in configs:
            file_dict = configs[file]
            file_extracted = False

            if file_dict['enabled']:
                # Use the value of use_name as the name of the directory
                output_dir = os.path.join(EXTRACTED_FOOTAGE_DIR, file_dict['use_name'])

                # Check if the directory already exists. If it does, do not extract footage
                # and print a message notifying the user the file was skipped.
                if os.path.isdir(output_dir):
                    print(f"{file_dict['use_name']} already exists! This file was skipped.")
                    continue

                os.mkdir(output_dir)

                # extracting file_name
                file_name = file[::-1].replace('_', '.', 1)[::-1]
                footage_path = os.path.join(directory, file_name)
                extracted_footage_path = os.path.join(EXTRACTED_FILES_DIR, file_name)

                if file_name.endswith('bag'):
                    for topic in file_dict:
                        if topic in ['enabled', 'use_name']:  # The keys 'enabled' and 'use_name' are not topics
                            continue

                        topic_dict = file_dict[topic]
                        if topic_dict['enabled']:

                            # creating topic directory
                            topic_dir = os.path.join(output_dir, topic_dict['use_name'])
                            os.mkdir(topic_dir)

                            self.extract_frames_from_bag(topic, topic_dict['use_name'],
                                                         footage_path, topic_dir, topic_dict['step_size'])

                            file_extracted = True

                else:  # if file is video
                    self.extract_frames_from_video(file_dict['use_name'], footage_path,
                                                   output_dir, file_dict['step_size'])

                    file_extracted = True

                # moving extracted file to EXTRACTED_FILES_DIR
                if file_extracted:
                    shutil.move(footage_path, extracted_footage_path)

    # TODO: Develop function that generates a roboflow upload config file
    #
    # The config file should have the following format. Each key is the name of a directory within
    # EXTRACTED_FOOTAGE_DIR. The value of each key indicates if the images in the directory should be uploaded
    # to Roboflow. Bag files have sub-keys controlling which topics should be uploaded. The value of batch_name
    # indicates the name of the batch to upload the images to, with default as the current date and time (see below).
    #
    # batch_name: "Uploaded on MM/DD/YYYY at HH:MM am/pm"
    # file_1_bag:
    #   image_topic_1: False
    #   image_topic_2: False
    #   ...
    # file_2_bag:
    #   image_topic_1: False
    #   image_topic_2: False
    #   ...
    # file_3_mp4: False
    # file_4_mov: False
    # ...
    def create_roboflow_config_file(self, directory):
        now = datetime.now()
        now_time = now.strftime("%m/%d/%y at %I:%M %p")
        batch_dict = {'batch_name': f"Uploaded on {now_time}"}
        extracted_directories = [name for name in os.listdir(directory) if os.path.isdir(os.path.join(directory, name))]

        for directory in extracted_directories:
            batch_dict[directory] = False

        yaml_string = yaml.dump(batch_dict)

        with open(ROBOFLOW_UPLOAD_CONFIG_FILE, "w") as yaml_file:
            yaml_file.write(yaml_string)

    # TODO: Develop function that takes a roboflow upload config file and uploads the desired images to Roboflow
    # Use the function upload_images_to_roboflow developed above. For each directory, ask the user to confirm they want
    # to upload the images in that directory before doing so. Use the API key and project ID
    # specified in ROBOFLOW_PROJECT_CONFIG_FILE when creating the project instance to pass to upload_images_to_roboflow.
    def upload_images_to_roboflow_with_config(self, directory, config_file):
        with open(ROBOFLOW_PROJECT_CONFIG_FILE, 'r') as project_config_file:
            project_configs = yaml.safe_load(project_config_file)

        rf = roboflow.Roboflow(api_key=project_configs['api_key'])
        project = rf.project(project_configs['project_id'])

        with open(config_file, 'r') as upload_config_file:
            upload_configs = yaml.safe_load(upload_config_file)

        for extracted_directory in upload_configs:
            if extracted_directory == 'batch_name':     # The key 'batch_name' is not an extracted directory
                continue

            if upload_configs[extracted_directory]:
                inp = input(f"Are you sure you want to upload the images in {extracted_directory}? (Y/N): ")
                if inp.lower() == "y":
                    extracted_directory_path = os.path.join(directory, extracted_directory)

                    # for video files, upload all images in the main extracted frames directory
                    uploaded_dir_path = os.path.join(UPLOADED_FOOTAGE, extracted_directory)
                    self.upload_images_to_roboflow(project, extracted_directory_path,
                                                   upload_configs['batch_name'], uploaded_dir_path)


if __name__ == '__main__':

    # Using command line arguments, check if the --generate_config flag is set
    # If it is, call the create_footage_extraction_config_file function
    # If it is not, ask the user to confirm they want to extract footage on the command line
    # If user confirms, call the extract_footage_with_config function

    # TODO: Change type of config file generated based on value of --generate-config <str_value>
    # If value is "footage", then generate a footage extraction config
    # If value is "roboflow", then generate a roboflow upload config
    # If no value is given for the --generate-config flag, then generate a footage extraction config

    # TODO: Check if the --default-bools <bool_value> flag is set. (Only used when --generate-config is also set)
    # If -defaults is set to True, generate a config file with all boolean values set to true
    # If -defaults is set to False or not specified, generate a config file with all boolean values set to false

    # TODO: Check if the --step-size <int_value> flag is set. (Only used when --generate-config is also set)
    # If --step-size is set, set the step size for all topics and video files to the specified value
    # If --step-size is not set, set the step size for all topics and video files to 10

    # TODO: Check if the --upload-to-roboflow flag is present. If so, use the roboflow config file to upload footage

    # TODO: Add comments to all functions explaining what they do (use first two functions as examples)
    # TODO: Remove all unncessary comments (TOODs, directions, examples, etc.)

    footage_extractor = FootageExtractor()

    arg_list = sys.argv[1:]

    generate_config_flag = False
    default_bools_flag = False
    step_size = 10
    upload_to_roboflow_flag = False
    roboflow_found = False

    for index, arg in enumerate(arg_list):
        if arg == '--generate-config':
            generate_config_flag = True
            if index+1 < len(arg_list) and arg_list[index+1].startswith("roboflow"):
                roboflow_found = True

        if arg == '--default-bools':
            if index+1 < len(arg_list):
                if arg_list[index+1].lower() == "true":
                    default_bools_flag = True

        if arg == '--step-size':
            if index+1 < len(arg_list):
                step_size = int(arg_list[index+1])

        if arg == '--upload-to-roboflow':
            upload_to_roboflow_flag = True

    if generate_config_flag:
        if roboflow_found:
            footage_extractor.create_roboflow_config_file(EXTRACTED_FOOTAGE_DIR)

        else:
            footage_extractor.create_footage_extraction_config_file(FOOTAGE_DIR,
                                                                    enabled=default_bools_flag, step_size=step_size)

    elif upload_to_roboflow_flag:
        footage_extractor.upload_images_to_roboflow_with_config(EXTRACTED_FOOTAGE_DIR, ROBOFLOW_UPLOAD_CONFIG_FILE)

    else:
        inp = input("Are you sure you want to extract footage? (Y/N): ")
        if inp.lower() == "y":
            footage_extractor.extract_footage_with_config(FOOTAGE_DIR, FOOTAGE_EXTRACTION_CONFIG_FILE)
