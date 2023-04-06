# CV Footage Extraction

## Purpose
These files are used for extracting frames from video (`.avi`, `.mov`, etc.) and rosbag files (`.bag`).

## Usage
1. Create the directory `footage_extraction/footage` and upload all video files there
2. Run
   ```
   python3 extract_footage.py --generate-config footage
   ```
- Additional flags include:
    - --default-bools: set all boolean values to --default-bools. Default "False"
    - --step-size: default 10
1. Modify `footage_extraction_config.yaml`
- Set `enabled` to true for any files/topics that you want to extract
- Set the `step_size` such that contiguous frames are not very similar
- You can also modify `use_name` to modify the outputted file name
1. Run 
   ```python3 extract_footage.py``` to extract the desired frames to `extracted_footage`
- The original file will be moved to `extracted_files`
1. Filter the extracted frames
2. Run `python3 extract_footage.py --generate-config robosub` to generate a Roboflow upload config and Roboflow project config
3. In `roboflow_project_config.yaml` 
- `api_key`: Log in to Roboflow, then go to your name in the top right, Settings, the desired workspace, and Roboflow API. Your Private API Key should be listed.
- `project_id` Log in to Roboflow, click the 3 dots next to the desired project, click `Rename Project`. Your Project ID should be listed.
1. In `roboflow_upload_config.yaml` set the desired files you want to upload to Roboflow to `true` and change the `batch_name` to the 
2. Run `python3 extract_footage.py --upload-to-roboflow` to upload the frames to Roboflow.