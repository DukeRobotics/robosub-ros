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
UPLOADED_FOOTAGE = os.path.join(FOOTAGE_EXTRACTION_DIR, 'uploaded_footage')
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
        Extract frames from the rosbag file and save them into output_dir

        :param footage_path: Path to rosbag file.
        :param topic_name: Name of the topic where the image feed is published. Can find by calling view_rosbag_info
        on the rosbag file.
        :param topic_use_name: Topic name that is used in the extracted frame names. Can set different from topic_name.
        :param output_dir: Path to directory to save the output. Will create a subdirectory within output_dir for each
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
        Extract frames from the video file and save them into output_dir

        :param use_name: Name that is used in the extracted frame names. Can be set different from video name.
        :param video_path: Path to video file.
        :param topic_name: Name of the topic where the image feed is published. Can find by calling view_rosbag_info
        on the rosbag file.
        :param output_dir: Path to directory to save the output. Will create a subdirectory within output_dir for each
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

    def upload_images_to_roboflow(self, rf_project, directory, batch_name, uploaded_dir):
        """
        Upload all images in directory to the Roboflow project rf_project under batch_name
        then move all successfully uploaded images to uploaded_dir

        :param rf_project: Name of Roboflow project
        :param directory: Path to directory containing all the images to be uploaded
        :param batch_name: Name of batch under rf_project to upload the images to
        :param uploaded_dir: Path to directory to move all successfully uploaded images to
        :return: Dictionary containing a list of successful images and a list of unsuccessful images
        """

        images = os.listdir(directory)
        status = {
            'successful': [],
            'unsuccessful': []
        }

        for image in images:
            image_path = os.path.join(directory, image)

            success = self.upload_images_to_roboflow_with_success(rf_project, image_path, batch_name)

            if success:
                status['successful'].append(image)
                shutil.move(image_path, uploaded_dir)
            else:
                status['unsuccessful'].append(image)

        return status

    def upload_images_to_roboflow_with_success(self, rf_project, image_path, batch_name):
        """
        Upload a single image at image_path to the Roboflow project rf_project under batch_name
        and return a success boolean

        :param rf_project: Name of Roboflow project
        :param image_path: Path to image to be uploaded
        :param batch_name: Name of batch under rf_project to upload the images to
        :return: Boolean indicating whether image upload is successful
        """

        if not os.path.isfile(image_path):
            print(f"ERROR: The provided image path {image_path} is not a valid path. Image failed to upload!")
            return False

        elif not rf_project.check_valid_image(image_path):
            print(f"ERROR: The image at {image_path} is not a supported file format.",
                  "Only PNG and JPEG files are supported. Image failed to upload!")
            return False

        return rf_project.single_upload(image_path=image_path, batch_name=batch_name)

    def create_footage_extraction_config_file(self, directory, enabled=False, step_size=10):
        """
        Prepare a yaml config file for footage extraction based on the list of files in directory
        and save the file at footage_extraction/footage_extraction_config.yaml

        :param directory: Path to directory containing the files to be extracted
        :param enabled: Default enabled attribute for every file and topic (for bag files). Set to False by default.
        :param step_size: Default step_size attribute for every file and topic (for bag files). Set to 10 by default.
        """

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

    def extract_footage_with_config(self, directory, config_file):
        """
        Read a footage extraction yaml config_file and extract footage in directory accordingly
        then move all extracted files into footage_extraction/extracted_files

        :param directory: Path to directory containing the files to be extracted
        :param config_file: Path to yaml config file
        """

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

                            # creating topic directory for the first time
                            if not os.path.isdir(output_dir):
                                os.mkdir(output_dir)

                            topic_dir = os.path.join(output_dir, topic_dict['use_name'])
                            if not os.path.isdir(topic_dir):
                                os.mkdir(topic_dir)

                            self.extract_frames_from_bag(topic, topic_dict['use_name'],
                                                         footage_path, topic_dir, topic_dict['step_size'])

                            file_extracted = True

                else:  # if file is video
                    os.mkdir(output_dir)
                    self.extract_frames_from_video(file_dict['use_name'], footage_path,
                                                   output_dir, file_dict['step_size'])

                    file_extracted = True

                # moving extracted file to EXTRACTED_FILES_DIR
                if file_extracted:
                    shutil.move(footage_path, extracted_footage_path)
                else:
                    print(f"Since no topics are enabled, the file {file_name} was not extracted.")

    def create_roboflow_upload_config_file(self, directory):
        """
        Prepare a yaml config file for uploading to Roboflow based on the list of extracted image folders in directory
        and save the file at footage_extraction/roboflow_upload_config.yaml

        :param directory: Path to directory containing the extracted image folders
        """

        now = datetime.now()
        now_time = now.strftime("%m-%d-%y at %I-%M %p")
        batch_dict = {'batch_name': f"Uploaded on {now_time}"}
        extracted_directories = [name for name in os.listdir(directory) if os.path.isdir(os.path.join(directory, name))]

        for extracted_directory in extracted_directories:
            file_dict = {}
            extracted_directory_path = os.path.join(directory, extracted_directory)

            for item in os.listdir(extracted_directory_path):
                if os.path.isdir(os.path.join(extracted_directory_path, item)):
                    file_dict[item] = False

            # if the extracted_directory_path comes from a bag file, there will be
            # some topic directories in extracted_directory_path
            if len(file_dict) > 0:
                batch_dict[extracted_directory] = file_dict

            # if the extracted_directory_path comes from a video file
            else:
                batch_dict[extracted_directory] = False

        yaml_string = yaml.dump(batch_dict)

        with open(ROBOFLOW_UPLOAD_CONFIG_FILE, "w") as yaml_file:
            yaml_file.write(yaml_string)

    def create_roboflow_project_config_file(self):
        """
        Prepare a yaml config file for uploading to Roboflow and save the file at
        footage_extraction/roboflow_project_config.yaml
        """

        dict = {
            "api_key": None,
            "project_id": None
        }

        yaml_string = yaml.dump(dict)

        with open(ROBOFLOW_PROJECT_CONFIG_FILE, "w") as yaml_file:
            yaml_file.write(yaml_string)

    def upload_images_to_roboflow_with_config(self, directory, config_file):
        """
        Read a Roboflow upload yaml config_file and upload images in directory to Roboflow accordingly
        then move all successfully uploaded images to footage_extraction/uploaded_footage

        :param directory: Path to directory containing the extracted image folders to be uploaded
        :param config_file: Path to yaml config file
        """

        with open(ROBOFLOW_PROJECT_CONFIG_FILE, 'r') as project_config_file:
            project_configs = yaml.safe_load(project_config_file)

        rf = roboflow.Roboflow(api_key=project_configs['api_key'])
        project = rf.project(project_configs['project_id'])

        with open(config_file, 'r') as upload_config_file:
            upload_configs = yaml.safe_load(upload_config_file)

        for extracted_directory in upload_configs:
            if extracted_directory == 'batch_name':     # The key 'batch_name' is not an extracted directory
                continue

            # creating directory for uploaded files
            extracted_directory_path = os.path.join(directory, extracted_directory)
            uploaded_dir_path = os.path.join(UPLOADED_FOOTAGE, extracted_directory)

            # if the extracted_directory comes from a video file, there are no topic subdirectories
            if isinstance(upload_configs[extracted_directory], bool):
                if upload_configs[extracted_directory]:
                    inp = input(f"Are you sure you want to upload the images in {extracted_directory}? (Y/N): ")
                    if inp.lower() == "y":
                        if not os.path.isdir(uploaded_dir_path):
                            os.mkdir(uploaded_dir_path)

                        status = self.upload_images_to_roboflow(project, extracted_directory_path,
                                                                upload_configs['batch_name'], uploaded_dir_path)
                        print(f"Successfully uploaded {len(status['successful'])} images,",
                              f"failed to upload {len(status['unsuccessful'])} images from {extracted_directory}",
                              f"under batch {upload_configs['batch_name']}")

            # if the extracted_directory comes from a bag file
            else:
                file_dict = upload_configs[extracted_directory]
                for topic in file_dict:
                    extracted_topic_path = os.path.join(extracted_directory_path, topic)
                    uploaded_topic_dir_path = os.path.join(uploaded_dir_path, topic)

                    if file_dict[topic]:
                        inp = input("Are you sure you want to upload the " +
                                    f"images in {extracted_directory}/{topic}? (Y/N): ")
                        if inp.lower() == "y":

                            # creating directory for the file and topic subdirectories for uploaded images (if required)
                            if not os.path.isdir(uploaded_dir_path):
                                os.mkdir(uploaded_dir_path)

                            if not os.path.isdir(uploaded_topic_dir_path):
                                os.mkdir(uploaded_topic_dir_path)

                            status = self.upload_images_to_roboflow(project, extracted_topic_path,
                                                                    upload_configs['batch_name'],
                                                                    uploaded_topic_dir_path)
                            print(f"Successfully uploaded {len(status['successful'])} images,",
                                  f"failed to upload {len(status['unsuccessful'])} images",
                                  f"from {extracted_directory}/{topic}",
                                  f"under batch {upload_configs['batch_name']}")

                if not os.path.isdir(uploaded_dir_path):
                    print(f"Since no topics are enabled, the directory {extracted_directory} was not uploaded.")

    def generate_folder_structure(self):
        if not os.path.isdir(FOOTAGE_DIR):
            os.mkdir(FOOTAGE_DIR)
        if not os.path.isdir(EXTRACTED_FOOTAGE_DIR):
            os.mkdir(EXTRACTED_FOOTAGE_DIR)
        if not os.path.isdir(EXTRACTED_FILES_DIR):
            os.mkdir(EXTRACTED_FILES_DIR)
        if not os.path.isdir(UPLOADED_FOOTAGE):
            os.mkdir(UPLOADED_FOOTAGE)


if __name__ == '__main__':
    """
    default: extract footage using the footage extraction config file
    --generate-config: default "footage"
        roboflow: generate a Roboflow upload and project config
        footage: generate a footage extraction config
            --default-bools: set all boolean values to --default-bools. Default "False"
            --step-size: default 10
    --upload-to-roboflow: upload footage to Roboflow using the Roboflow upload config file
    """

    footage_extractor = FootageExtractor()

    footage_extractor.generate_folder_structure()

    arg_list = sys.argv[1:]

    generate_config_flag = False
    default_bools_flag = False
    step_size = 10
    upload_to_roboflow_flag = False
    roboflow_found = False

    for index, arg in enumerate(arg_list):
        if arg == '--generate-config':
            generate_config_flag = True
            if index+1 < len(arg_list) and arg_list[index+1] == "roboflow":
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
            footage_extractor.create_roboflow_upload_config_file(EXTRACTED_FOOTAGE_DIR)

            if not os.path.isfile(ROBOFLOW_PROJECT_CONFIG_FILE):
                footage_extractor.create_roboflow_project_config_file()

        else:
            footage_extractor.create_footage_extraction_config_file(FOOTAGE_DIR,
                                                                    enabled=default_bools_flag, step_size=step_size)

    elif upload_to_roboflow_flag:
        footage_extractor.upload_images_to_roboflow_with_config(EXTRACTED_FOOTAGE_DIR, ROBOFLOW_UPLOAD_CONFIG_FILE)

    else:
        inp = input("Are you sure you want to extract footage? (Y/N): ")
        if inp.lower() == "y":
            footage_extractor.extract_footage_with_config(FOOTAGE_DIR, FOOTAGE_EXTRACTION_CONFIG_FILE)
