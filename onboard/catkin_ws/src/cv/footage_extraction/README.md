# CV Footage Extraction

## Purpose
These files are used for extracting frames from video files (`.avi`, `.mov`, etc.) and rosbag files (`.bag`) and uploading these extracted frames to Roboflow.

## Usage
1. Create the directory `footage_extraction/footage` and upload all files you want to extract here
2. Run
   ```
   python3 extract_footage.py --generate-config footage
   ```
- Optional flags:
    - --default-bools: set all boolean values to --default-bools. Default "False"
    - --step-size: default 10
3. Modify `footage_extraction_config.yaml`
- Set `enabled` to true for any files/topics that you want to extract
- Set the `step_size` such that there is enough motion between frames
- You can also modify `use_name` to change the extracted file names
4. Extract the frames by running
   ```
   python3 extract_footage.py
   ```
- Note: the original files will be moved to `extracted_files`
5. Filter the extracted frames in `extracted_footage`
6. Run `python3 extract_footage.py --generate-config robosub` to generate a Roboflow upload config and Roboflow project config
7. In `roboflow_project_config.yaml` update the following:
- `api_key`: Log in to Roboflow, then go to your name in the top right, Settings, the desired workspace, and Roboflow API. Copy/paste your Private API Key.
- `project_id`: Log in to Roboflow, click the 3 dots next to the desired project, click Rename Project. Copy/paste your Project ID.
8. In `roboflow_upload_config.yaml` set the desired files you want to upload to Roboflow to `true` and update the `batch_name`.
9. Run `python3 extract_footage.py --upload-to-roboflow` to upload the frames to Roboflow.