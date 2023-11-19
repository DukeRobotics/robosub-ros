/* eslint-disable */
// These files were generated using "ros-typescript-generator"
export interface CustomMsgsAcousticsDataActionFeedback {
  curr_stage: number;
  total_stages: number;
  message: string;
}

export interface CustomMsgsAcousticsDataActionGoal {
  samp_f: number;
  tar_f: number;
  hydrophone_set: CustomMsgsHydrophoneSet;
  location: GeometryMsgsPoint;
}

export interface CustomMsgsAcousticsDataActionResult {
  file_paths: string[];
}

export interface CustomMsgsAcousticsGuessActionFeedback {
  curr_stage: number;
  total_stages: number;
  message: string;
}

export interface CustomMsgsAcousticsGuessActionGoal {
  file_paths: string[];
  samp_f: number;
  tar_f: number;
}

export interface CustomMsgsAcousticsGuessActionResult {
  guess: GeometryMsgsPoint;
}

export interface CustomMsgsAcousticsProcessingActionFeedback {
  total_count: number;
  process_count: number;
  success_count: number;
}

export interface CustomMsgsAcousticsProcessingActionGoal {
  file_paths: string[];
  if_double: boolean;
  samp_f: number;
  tar_f: number;
  guess: GeometryMsgsPoint;
}

export interface CustomMsgsAcousticsProcessingActionResult {
  hz_angle: number;
  total_count: number;
  success_count: number;
  valid_count: number;
}

export interface CustomMsgsAcousticsWrapperActionFeedback {
  curr_stage: number;
  total_stages: number;
  message: string;
}

export interface CustomMsgsAcousticsWrapperActionGoal {
  tar_f: number;
  location: GeometryMsgsPoint;
}

export interface CustomMsgsAcousticsWrapperActionResult {
  hz_angle: number;
}

export interface CustomMsgsConnectDepthAiCameraResponse {
  success: boolean;
}

export interface CustomMsgsConnectUsbCameraRequest {
  channel: number;
}

export interface CustomMsgsConnectUsbCameraResponse {
  success: boolean;
}

export interface CustomMsgsCvObject {
  header: StdMsgsHeader;
  xmin: number;
  ymin: number;
  xmax: number;
  ymax: number;
  yaw: number;
  coords: GeometryMsgsPoint;
  height: number;
  width: number;
  distance: number;
  label: string;
  score: number;
  sonar: boolean;
}

export interface CustomMsgsDvlRaw {
  header: StdMsgsHeader;
  sa_roll: number;
  sa_pitch: number;
  sa_heading: number;
  ts_salinity: number;
  ts_temperature: number;
  ts_depth: number;
  ts_sound_speed: number;
  ts_built_in_test: number;
  bi_x_axis: number;
  bi_y_axis: number;
  bi_z_axis: number;
  bi_error: number;
  bi_status: string;
  bs_transverse: number;
  bs_longitudinal: number;
  bs_normal: number;
  bs_status: string;
  be_east: number;
  be_north: number;
  be_upwards: number;
  be_status: string;
  bd_east: number;
  bd_north: number;
  bd_upwards: number;
  bd_range: number;
  bd_time: number;
}

export interface CustomMsgsEnableModelRequest {
  model_name: string;
  enabled: boolean;
}

export interface CustomMsgsEnableModelResponse {
  success: boolean;
}

export interface CustomMsgsGetRunningNodesResponse {
  running_nodes_msgs: CustomMsgsRunningNode[];
}

export interface CustomMsgsHydrophoneSet {
  type: number;
}

export enum CustomMsgsHydrophoneSetConst {
  GUESS = 0,
  PROCESS = 1,
}

export interface CustomMsgsMemory {
  used: number;
  total: number;
  percentage: number;
}

export interface CustomMsgsRemoteLaunchInfo {
  msg_type: number;
  running_node_info: CustomMsgsRunningNode;
}

export enum CustomMsgsRemoteLaunchInfoConst {
  EXECUTING = 0,
  TERMINATING = 1,
}

export interface CustomMsgsRunningNode {
  pid: number;
  package: string;
  file: string;
  args: string[];
  file_type: number;
}

export enum CustomMsgsRunningNodeConst {
  ROSLAUNCH = 0,
  ROSRUN = 1,
}

export interface CustomMsgsSaleaeActionFeedback {
  curr_stage: number;
  total_stages: number;
  message: string;
}

export interface CustomMsgsSaleaeActionGoal {
  hydrophone_set: CustomMsgsHydrophoneSet;
  capture_count: number;
  capture_duration: number;
}

export interface CustomMsgsSaleaeActionResult {
  file_paths: string[];
}

export interface CustomMsgsServoAngleArray {
  angles: number[];
}

export interface CustomMsgsSetServoRequest {
  num: number;
  angle: number;
}

export interface CustomMsgsSetServoResponse {
  success: boolean;
}

export interface CustomMsgsSimObject {
  label: string;
  distance: number;
  points: GeometryMsgsPoint[];
}

export interface CustomMsgsSimObjectArray {
  objects: CustomMsgsSimObject[];
}

export interface CustomMsgsSonarRequest {
  header: StdMsgsHeader;
  type: string;
  center_degrees: number;
  breadth_degrees: number;
  depth: number;
}

export interface CustomMsgsSonarResponse {
  header: StdMsgsHeader;
  type: string;
  x: number;
  y: number;
}

export interface CustomMsgsStartLaunchRequest {
  package: string;
  file: string;
  args: string[];
  is_launch_file: boolean;
}

export interface CustomMsgsStartLaunchResponse {
  pid: number;
}

export interface CustomMsgsStopLaunchRequest {
  pid: number;
}

export interface CustomMsgsStopLaunchResponse {
  success: boolean;
}

export interface CustomMsgsSweepActionFeedback {
  current_angle: number;
}

export interface CustomMsgsSweepActionGoal {
  start_angle: number;
  end_angle: number;
  center_z_angle: number;
  distance_of_scan: number;
}

export interface CustomMsgsSweepActionResult {
  x_pos: number;
  y_pos: number;
}

export interface CustomMsgsSystemUsage {
  cpu_percent: number;
  cpu_speed: number;
  gpu_percent: number;
  gpu_speed: number;
  gpu_memory: CustomMsgsMemory;
  ram: CustomMsgsMemory;
  disk: CustomMsgsMemory;
}

export interface CustomMsgsThrusterSpeeds {
  header: StdMsgsHeader;
  speeds: number[];
}