/* eslint-disable */
// All ROS 1 messages, services, and actions are defined here.
export enum BondConstantsConst {
  DEAD_PUBLISH_PERIOD = 0.05,
  DEFAULT_CONNECT_TIMEOUT = 10,
  DEFAULT_HEARTBEAT_TIMEOUT = 4,
  DEFAULT_DISCONNECT_TIMEOUT = 2,
  DEFAULT_HEARTBEAT_PERIOD = 1,
  DISABLE_HEARTBEAT_TIMEOUT_PARAM = '/bond_disable_heartbeat_timeout',
}

export enum DynamicReconfigureSensorLevelsConst {
  RECONFIGURE_CLOSE = 3,
  RECONFIGURE_STOP = 1,
  RECONFIGURE_RUNNING = 0,
}

export interface ActionlibMsgsGoalId {
  stamp: { secs: number, nsecs: number };
  id: string;
}

export interface ActionlibMsgsGoalStatus {
  goal_id: ActionlibMsgsGoalId;
  status: number;
  text: string;
}

export enum ActionlibMsgsGoalStatusConst {
  PENDING = 0,
  ACTIVE = 1,
  PREEMPTED = 2,
  SUCCEEDED = 3,
  ABORTED = 4,
  REJECTED = 5,
  PREEMPTING = 6,
  RECALLING = 7,
  RECALLED = 8,
  LOST = 9,
}

export interface ActionlibMsgsGoalStatusArray {
  header: StdMsgsHeader;
  status_list: ActionlibMsgsGoalStatus[];
}

export interface ActionlibTestAction {
  action_goal: ActionlibTestActionGoal;
  action_result: ActionlibTestActionResult;
  action_feedback: ActionlibTestActionFeedback;
}

export interface ActionlibTestActionFeedback {
  header: StdMsgsHeader;
  status: ActionlibMsgsGoalStatus;
  feedback: ActionlibTestFeedback;
}

export interface ActionlibTestActionGoal {
  header: StdMsgsHeader;
  goal_id: ActionlibMsgsGoalId;
  goal: ActionlibTestGoal;
}

export interface ActionlibTestActionResult {
  header: StdMsgsHeader;
  status: ActionlibMsgsGoalStatus;
  result: ActionlibTestResult;
}

export interface ActionlibTestFeedback {
  feedback: number;
}

export interface ActionlibTestGoal {
  goal: number;
}

export interface ActionlibTestRequestAction {
  action_goal: ActionlibTestRequestActionGoal;
  action_result: ActionlibTestRequestActionResult;
  action_feedback: ActionlibTestRequestActionFeedback;
}

export interface ActionlibTestRequestActionFeedback {
  header: StdMsgsHeader;
  status: ActionlibMsgsGoalStatus;
}

export interface ActionlibTestRequestActionGoal {
  header: StdMsgsHeader;
  goal_id: ActionlibMsgsGoalId;
  goal: ActionlibTestRequestGoal;
}

export interface ActionlibTestRequestActionGoal {
  terminate_status: number;
  ignore_cancel: boolean;
  result_text: string;
  the_result: number;
  is_simple_client: boolean;
  delay_accept: { secs: number, nsecs: number };
  delay_terminate: { secs: number, nsecs: number };
  pause_status: { secs: number, nsecs: number };
}

export enum ActionlibTestRequestActionGoalConst {
  TERMINATE_SUCCESS = 0,
  TERMINATE_ABORTED = 1,
  TERMINATE_REJECTED = 2,
  TERMINATE_LOSE = 3,
  TERMINATE_DROP = 4,
  TERMINATE_EXCEPTION = 5,
}

export interface ActionlibTestRequestActionResult {
  header: StdMsgsHeader;
  status: ActionlibMsgsGoalStatus;
  result: ActionlibTestRequestResult;
}

export interface ActionlibTestRequestActionResult {
  the_result: number;
  is_simple_server: boolean;
}

export interface ActionlibTestRequestGoal {
  terminate_status: number;
  ignore_cancel: boolean;
  result_text: string;
  the_result: number;
  is_simple_client: boolean;
  delay_accept: { secs: number, nsecs: number };
  delay_terminate: { secs: number, nsecs: number };
  pause_status: { secs: number, nsecs: number };
}

export enum ActionlibTestRequestGoalConst {
  TERMINATE_SUCCESS = 0,
  TERMINATE_ABORTED = 1,
  TERMINATE_REJECTED = 2,
  TERMINATE_LOSE = 3,
  TERMINATE_DROP = 4,
  TERMINATE_EXCEPTION = 5,
}

export interface ActionlibTestRequestResult {
  the_result: number;
  is_simple_server: boolean;
}

export interface ActionlibTestResult {
  result: number;
}

export interface ActionlibTwoIntsAction {
  action_goal: ActionlibTwoIntsActionGoal;
  action_result: ActionlibTwoIntsActionResult;
  action_feedback: ActionlibTwoIntsActionFeedback;
}

export interface ActionlibTwoIntsActionFeedback {
  header: StdMsgsHeader;
  status: ActionlibMsgsGoalStatus;
}

export interface ActionlibTwoIntsActionGoal {
  a: number;
  b: number;
}

export interface ActionlibTwoIntsActionGoal {
  header: StdMsgsHeader;
  goal_id: ActionlibMsgsGoalId;
  goal: ActionlibTwoIntsGoal;
}

export interface ActionlibTwoIntsActionResult {
  header: StdMsgsHeader;
  status: ActionlibMsgsGoalStatus;
  result: ActionlibTwoIntsResult;
}

export interface ActionlibTwoIntsActionResult {
  sum: number;
}

export interface ActionlibTwoIntsGoal {
  a: number;
  b: number;
}

export interface ActionlibTwoIntsResult {
  sum: number;
}

export interface BondStatus {
  header: StdMsgsHeader;
  id: string;
  instance_id: string;
  active: boolean;
  heartbeat_timeout: number;
  heartbeat_period: number;
}

export interface DiagnosticMsgsAddDiagnosticsRequest {
  load_namespace: string;
}

export interface DiagnosticMsgsAddDiagnosticsResponse {
  success: boolean;
  message: string;
}

export interface DiagnosticMsgsDiagnosticArray {
  header: StdMsgsHeader;
  status: DiagnosticMsgsDiagnosticStatus[];
}

export interface DiagnosticMsgsDiagnosticStatus {
  level: number;
  name: string;
  message: string;
  hardware_id: string;
  values: GeographicMsgsKeyValue[];
}

export enum DiagnosticMsgsDiagnosticStatusConst {
  OK = 0,
  WARN = 1,
  ERROR = 2,
  STALE = 3,
}

export interface DiagnosticMsgsKeyValue {
  key: string;
  value: string;
}

export interface DiagnosticMsgsSelfTestResponse {
  id: string;
  passed: number;
  status: DiagnosticMsgsDiagnosticStatus[];
}

export interface DynamicReconfigureBoolParameter {
  name: string;
  value: boolean;
}

export interface DynamicReconfigureConfig {
  bools: DynamicReconfigureBoolParameter[];
  ints: DynamicReconfigureIntParameter[];
  strs: DynamicReconfigureStrParameter[];
  doubles: DynamicReconfigureDoubleParameter[];
  groups: DynamicReconfigureGroupState[];
}

export interface DynamicReconfigureConfigDescription {
  groups: DynamicReconfigureGroup[];
  max: DynamicReconfigureConfig;
  min: DynamicReconfigureConfig;
  dflt: DynamicReconfigureConfig;
}

export interface DynamicReconfigureDoubleParameter {
  name: string;
  value: number;
}

export interface DynamicReconfigureGroup {
  name: string;
  type: string;
  parameters: DynamicReconfigureParamDescription[];
  parent: number;
  id: number;
}

export interface DynamicReconfigureGroupState {
  name: string;
  state: boolean;
  id: number;
  parent: number;
}

export interface DynamicReconfigureIntParameter {
  name: string;
  value: number;
}

export interface DynamicReconfigureParamDescription {
  name: string;
  type: string;
  level: number;
  description: string;
  edit_method: string;
}

export interface DynamicReconfigureReconfigureRequest {
  config: DynamicReconfigureConfig;
}

export interface DynamicReconfigureReconfigureResponse {
  config: DynamicReconfigureConfig;
}

export interface DynamicReconfigureStrParameter {
  name: string;
  value: string;
}

export interface GeographicMsgsBoundingBox {
  min_pt: GeographicMsgsGeoPoint;
  max_pt: GeographicMsgsGeoPoint;
}

export interface GeographicMsgsGeoPath {
  header: StdMsgsHeader;
  poses: GeographicMsgsGeoPoseStamped[];
}

export interface GeographicMsgsGeoPoint {
  latitude: number;
  longitude: number;
  altitude: number;
}

export interface GeographicMsgsGeoPointStamped {
  header: StdMsgsHeader;
  position: GeographicMsgsGeoPoint;
}

export interface GeographicMsgsGeoPose {
  position: GeographicMsgsGeoPoint;
  orientation: GeometryMsgsQuaternion;
}

export interface GeographicMsgsGeoPoseStamped {
  header: StdMsgsHeader;
  pose: GeographicMsgsGeoPose;
}

export interface GeographicMsgsGeographicMap {
  header: StdMsgsHeader;
  id: UuidMsgsUniqueId;
  bounds: GeographicMsgsBoundingBox;
  points: GeographicMsgsWayPoint[];
  features: GeographicMsgsMapFeature[];
  props: GeographicMsgsKeyValue[];
}

export interface GeographicMsgsGeographicMapChanges {
  header: StdMsgsHeader;
  diffs: GeographicMsgsGeographicMap;
  deletes: UuidMsgsUniqueId[];
}

export interface GeographicMsgsGetGeoPathRequest {
  start: GeographicMsgsGeoPoint;
  goal: GeographicMsgsGeoPoint;
}

export interface GeographicMsgsGetGeoPathResponse {
  success: boolean;
  status: string;
  plan: GeographicMsgsGeoPath;
  network: UuidMsgsUniqueId;
  start_seg: UuidMsgsUniqueId;
  goal_seg: UuidMsgsUniqueId;
  distance: number;
}

export interface GeographicMsgsGetGeographicMapRequest {
  url: string;
  bounds: GeographicMsgsBoundingBox;
}

export interface GeographicMsgsGetGeographicMapResponse {
  success: boolean;
  status: string;
  map: GeographicMsgsGeographicMap;
}

export interface GeographicMsgsGetRoutePlanRequest {
  network: UuidMsgsUniqueId;
  start: UuidMsgsUniqueId;
  goal: UuidMsgsUniqueId;
}

export interface GeographicMsgsGetRoutePlanResponse {
  success: boolean;
  status: string;
  plan: GeographicMsgsRoutePath;
}

export interface GeographicMsgsKeyValue {
  key: string;
  value: string;
}

export interface GeographicMsgsMapFeature {
  id: UuidMsgsUniqueId;
  components: UuidMsgsUniqueId[];
  props: GeographicMsgsKeyValue[];
}

export interface GeographicMsgsRouteNetwork {
  header: StdMsgsHeader;
  id: UuidMsgsUniqueId;
  bounds: GeographicMsgsBoundingBox;
  points: GeographicMsgsWayPoint[];
  segments: GeographicMsgsRouteSegment[];
  props: GeographicMsgsKeyValue[];
}

export interface GeographicMsgsRoutePath {
  header: StdMsgsHeader;
  network: UuidMsgsUniqueId;
  segments: UuidMsgsUniqueId[];
  props: GeographicMsgsKeyValue[];
}

export interface GeographicMsgsRouteSegment {
  id: UuidMsgsUniqueId;
  start: UuidMsgsUniqueId;
  end: UuidMsgsUniqueId;
  props: GeographicMsgsKeyValue[];
}

export interface GeographicMsgsUpdateGeographicMapRequest {
  updates: GeographicMsgsGeographicMapChanges;
}

export interface GeographicMsgsUpdateGeographicMapResponse {
  success: boolean;
  status: string;
}

export interface GeographicMsgsWayPoint {
  id: UuidMsgsUniqueId;
  position: GeographicMsgsGeoPoint;
  props: GeographicMsgsKeyValue[];
}

export interface GeometryMsgsAccel {
  linear: GeometryMsgsVector3;
  angular: GeometryMsgsVector3;
}

export interface GeometryMsgsAccelStamped {
  header: StdMsgsHeader;
  accel: GeometryMsgsAccel;
}

export interface GeometryMsgsAccelWithCovariance {
  accel: GeometryMsgsAccel;
  covariance: number[];
}

export interface GeometryMsgsAccelWithCovarianceStamped {
  header: StdMsgsHeader;
  accel: GeometryMsgsAccelWithCovariance;
}

export interface GeometryMsgsInertia {
  m: number;
  com: GeometryMsgsVector3;
  ixx: number;
  ixy: number;
  ixz: number;
  iyy: number;
  iyz: number;
  izz: number;
}

export interface GeometryMsgsInertiaStamped {
  header: StdMsgsHeader;
  inertia: GeometryMsgsInertia;
}

export interface GeometryMsgsPoint {
  x: number;
  y: number;
  z: number;
}

export interface GeometryMsgsPoint32 {
  x: number;
  y: number;
  z: number;
}

export interface GeometryMsgsPointStamped {
  header: StdMsgsHeader;
  point: GeometryMsgsPoint;
}

export interface GeometryMsgsPolygon {
  points: GeometryMsgsPoint32[];
}

export interface GeometryMsgsPolygonStamped {
  header: StdMsgsHeader;
  polygon: GeometryMsgsPolygon;
}

export interface GeometryMsgsPose {
  position: GeometryMsgsPoint;
  orientation: GeometryMsgsQuaternion;
}

export interface GeometryMsgsPose2D {
  x: number;
  y: number;
  theta: number;
}

export interface GeometryMsgsPoseArray {
  header: StdMsgsHeader;
  poses: GeometryMsgsPose[];
}

export interface GeometryMsgsPoseStamped {
  header: StdMsgsHeader;
  pose: GeometryMsgsPose;
}

export interface GeometryMsgsPoseWithCovariance {
  pose: GeometryMsgsPose;
  covariance: number[];
}

export interface GeometryMsgsPoseWithCovarianceStamped {
  header: StdMsgsHeader;
  pose: GeometryMsgsPoseWithCovariance;
}

export interface GeometryMsgsQuaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

export interface GeometryMsgsQuaternionStamped {
  header: StdMsgsHeader;
  quaternion: GeometryMsgsQuaternion;
}

export interface GeometryMsgsTransform {
  translation: GeometryMsgsVector3;
  rotation: GeometryMsgsQuaternion;
}

export interface GeometryMsgsTransformStamped {
  header: StdMsgsHeader;
  child_frame_id: string;
  transform: GeometryMsgsTransform;
}

export interface GeometryMsgsTwist {
  linear: GeometryMsgsVector3;
  angular: GeometryMsgsVector3;
}

export interface GeometryMsgsTwistStamped {
  header: StdMsgsHeader;
  twist: GeometryMsgsTwist;
}

export interface GeometryMsgsTwistWithCovariance {
  twist: GeometryMsgsTwist;
  covariance: number[];
}

export interface GeometryMsgsTwistWithCovarianceStamped {
  header: StdMsgsHeader;
  twist: GeometryMsgsTwistWithCovariance;
}

export interface GeometryMsgsVector3 {
  x: number;
  y: number;
  z: number;
}

export interface GeometryMsgsVector3Stamped {
  header: StdMsgsHeader;
  vector: GeometryMsgsVector3;
}

export interface GeometryMsgsWrench {
  force: GeometryMsgsVector3;
  torque: GeometryMsgsVector3;
}

export interface GeometryMsgsWrenchStamped {
  header: StdMsgsHeader;
  wrench: GeometryMsgsWrench;
}

export interface NavMsgsGetMapAction {
  action_goal: NavMsgsGetMapActionGoal;
  action_result: NavMsgsGetMapActionResult;
  action_feedback: NavMsgsGetMapActionFeedback;
}

export interface NavMsgsGetMapActionFeedback {
  header: StdMsgsHeader;
  status: ActionlibMsgsGoalStatus;
}

export interface NavMsgsGetMapActionGoal {
  header: StdMsgsHeader;
  goal_id: ActionlibMsgsGoalId;
}

export interface NavMsgsGetMapActionResult {
  header: StdMsgsHeader;
  status: ActionlibMsgsGoalStatus;
  result: NavMsgsGetMapResult;
}

export interface NavMsgsGetMapActionResult {
  map: NavMsgsOccupancyGrid;
}

export interface NavMsgsGetMapResponse {
  map: NavMsgsOccupancyGrid;
}

export interface NavMsgsGetMapResult {
  map: NavMsgsOccupancyGrid;
}

export interface NavMsgsGetPlanRequest {
  start: GeometryMsgsPoseStamped;
  goal: GeometryMsgsPoseStamped;
  tolerance: number;
}

export interface NavMsgsGetPlanResponse {
  plan: NavMsgsPath;
}

export interface NavMsgsGridCells {
  header: StdMsgsHeader;
  cell_width: number;
  cell_height: number;
  cells: GeometryMsgsPoint[];
}

export interface NavMsgsLoadMapRequest {
  map_url: string;
}

export interface NavMsgsLoadMapResponse {
  map: NavMsgsOccupancyGrid;
  result: number;
}

export enum NavMsgsLoadMapResponseConst {
  RESULT_SUCCESS = 0,
  RESULT_MAP_DOES_NOT_EXIST = 1,
  RESULT_INVALID_MAP_DATA = 2,
  RESULT_INVALID_MAP_METADATA = 3,
  RESULT_UNDEFINED_FAILURE = 255,
}

export interface NavMsgsMapMetaData {
  map_load_time: { secs: number, nsecs: number };
  resolution: number;
  width: number;
  height: number;
  origin: GeometryMsgsPose;
}

export interface NavMsgsOccupancyGrid {
  header: StdMsgsHeader;
  info: NavMsgsMapMetaData;
  data: number[];
}

export interface NavMsgsOdometry {
  header: StdMsgsHeader;
  child_frame_id: string;
  pose: GeometryMsgsPoseWithCovariance;
  twist: GeometryMsgsTwistWithCovariance;
}

export interface NavMsgsPath {
  header: StdMsgsHeader;
  poses: GeometryMsgsPoseStamped[];
}

export interface NavMsgsSetMapRequest {
  map: NavMsgsOccupancyGrid;
  initial_pose: GeometryMsgsPoseWithCovarianceStamped;
}

export interface NavMsgsSetMapResponse {
  success: boolean;
}

export interface NodeletNodeletListResponse {
  nodelets: string[];
}

export interface NodeletNodeletLoadRequest {
  name: string;
  type: string;
  remap_source_args: string[];
  remap_target_args: string[];
  my_argv: string[];
  bond_id: string;
}

export interface NodeletNodeletLoadResponse {
  success: boolean;
}

export interface NodeletNodeletUnloadRequest {
  name: string;
}

export interface NodeletNodeletUnloadResponse {
  success: boolean;
}

export interface RobotLocalizationFromLlRequest {
  ll_point: GeographicMsgsGeoPoint;
}

export interface RobotLocalizationFromLlResponse {
  map_point: GeometryMsgsPoint;
}

export interface RobotLocalizationGetStateRequest {
  time_stamp: { secs: number, nsecs: number };
  frame_id: string;
}

export interface RobotLocalizationGetStateResponse {
  state: number[];
  covariance: number[];
}

export interface RobotLocalizationSetDatumRequest {
  geo_pose: GeographicMsgsGeoPose;
}

export interface RobotLocalizationSetPoseRequest {
  pose: GeometryMsgsPoseWithCovarianceStamped;
}

export interface RobotLocalizationSetUtmZoneRequest {
  utm_zone: string;
}

export interface RobotLocalizationToLlRequest {
  map_point: GeometryMsgsPoint;
}

export interface RobotLocalizationToLlResponse {
  ll_point: GeographicMsgsGeoPoint;
}

export interface RobotLocalizationToggleFilterProcessingRequest {
  on: boolean;
}

export interface RobotLocalizationToggleFilterProcessingResponse {
  status: boolean;
}

export interface RoscppGetLoggersResponse {
  loggers: RoscppLogger[];
}

export interface RoscppLogger {
  name: string;
  level: string;
}

export interface RoscppSetLoggerLevelRequest {
  logger: string;
  level: string;
}

export interface RosgraphMsgsClock {
  clock: { secs: number, nsecs: number };
}

export interface RosgraphMsgsLog {
  header: StdMsgsHeader;
  level: number;
  name: string;
  msg: string;
  file: string;
  function: string;
  line: number;
  topics: string[];
}

export enum RosgraphMsgsLogConst {
  DEBUG = 1,
  INFO = 2,
  WARN = 4,
  ERROR = 8,
  FATAL = 16,
}

export interface RosgraphMsgsTopicStatistics {
  topic: string;
  node_pub: string;
  node_sub: string;
  window_start: { secs: number, nsecs: number };
  window_stop: { secs: number, nsecs: number };
  delivered_msgs: number;
  dropped_msgs: number;
  traffic: number;
  period_mean: { secs: number, nsecs: number };
  period_stddev: { secs: number, nsecs: number };
  period_max: { secs: number, nsecs: number };
  stamp_age_mean: { secs: number, nsecs: number };
  stamp_age_stddev: { secs: number, nsecs: number };
  stamp_age_max: { secs: number, nsecs: number };
}

export interface RosserialArduinoAdc {
  adc0: number;
  adc1: number;
  adc2: number;
  adc3: number;
  adc4: number;
  adc5: number;
}

export interface RosserialArduinoTestRequest {
  input: string;
}

export interface RosserialArduinoTestResponse {
  output: string;
}

export interface RosserialMsgsLog {
  level: number;
  msg: string;
}

export enum RosserialMsgsLogConst {
  ROSDEBUG = 0,
  INFO = 1,
  WARN = 2,
  ERROR = 3,
  FATAL = 4,
}

export interface RosserialMsgsRequestParamRequest {
  name: string;
}

export interface RosserialMsgsRequestParamResponse {
  ints: number[];
  floats: number[];
  strings: string[];
}

export interface RosserialMsgsTopicInfo {
  topic_id: number;
  topic_name: string;
  message_type: string;
  md5sum: string;
  buffer_size: number;
}

export enum RosserialMsgsTopicInfoConst {
  ID_PUBLISHER = 0,
  ID_SUBSCRIBER = 1,
  ID_SERVICE_SERVER = 2,
  ID_SERVICE_CLIENT = 4,
  ID_PARAMETER_REQUEST = 6,
  ID_LOG = 7,
  ID_TIME = 10,
  ID_TX_STOP = 11,
}

export interface SensorMsgsBatteryState {
  header: StdMsgsHeader;
  voltage: number;
  temperature: number;
  current: number;
  charge: number;
  capacity: number;
  design_capacity: number;
  percentage: number;
  power_supply_status: number;
  power_supply_health: number;
  power_supply_technology: number;
  present: boolean;
  cell_voltage: number[];
  cell_temperature: number[];
  location: string;
  serial_number: string;
}

export enum SensorMsgsBatteryStateConst {
  POWER_SUPPLY_STATUS_UNKNOWN = 0,
  POWER_SUPPLY_STATUS_CHARGING = 1,
  POWER_SUPPLY_STATUS_DISCHARGING = 2,
  POWER_SUPPLY_STATUS_NOT_CHARGING = 3,
  POWER_SUPPLY_STATUS_FULL = 4,
  POWER_SUPPLY_HEALTH_UNKNOWN = 0,
  POWER_SUPPLY_HEALTH_GOOD = 1,
  POWER_SUPPLY_HEALTH_OVERHEAT = 2,
  POWER_SUPPLY_HEALTH_DEAD = 3,
  POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4,
  POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5,
  POWER_SUPPLY_HEALTH_COLD = 6,
  POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7,
  POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8,
  POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0,
  POWER_SUPPLY_TECHNOLOGY_NIMH = 1,
  POWER_SUPPLY_TECHNOLOGY_LION = 2,
  POWER_SUPPLY_TECHNOLOGY_LIPO = 3,
  POWER_SUPPLY_TECHNOLOGY_LIFE = 4,
  POWER_SUPPLY_TECHNOLOGY_NICD = 5,
  POWER_SUPPLY_TECHNOLOGY_LIMN = 6,
}

export interface SensorMsgsCameraInfo {
  header: StdMsgsHeader;
  height: number;
  width: number;
  distortion_model: string;
  D: number[];
  K: number[];
  R: number[];
  P: number[];
  binning_x: number;
  binning_y: number;
  roi: SensorMsgsRegionOfInterest;
}

export interface SensorMsgsChannelFloat32 {
  name: string;
  values: number[];
}

export interface SensorMsgsCompressedImage {
  header: StdMsgsHeader;
  format: string;
  data: number[];
}

export interface SensorMsgsFluidPressure {
  header: StdMsgsHeader;
  fluid_pressure: number;
  variance: number;
}

export interface SensorMsgsIlluminance {
  header: StdMsgsHeader;
  illuminance: number;
  variance: number;
}

export interface SensorMsgsImage {
  header: StdMsgsHeader;
  height: number;
  width: number;
  encoding: string;
  is_bigendian: number;
  step: number;
  data: number[];
}

export interface SensorMsgsImu {
  header: StdMsgsHeader;
  orientation: GeometryMsgsQuaternion;
  orientation_covariance: number[];
  angular_velocity: GeometryMsgsVector3;
  angular_velocity_covariance: number[];
  linear_acceleration: GeometryMsgsVector3;
  linear_acceleration_covariance: number[];
}

export interface SensorMsgsJointState {
  header: StdMsgsHeader;
  name: string[];
  position: number[];
  velocity: number[];
  effort: number[];
}

export interface SensorMsgsJoy {
  header: StdMsgsHeader;
  axes: number[];
  buttons: number[];
}

export interface SensorMsgsJoyFeedback {
  type: number;
  id: number;
  intensity: number;
}

export enum SensorMsgsJoyFeedbackConst {
  TYPE_LED = 0,
  TYPE_RUMBLE = 1,
  TYPE_BUZZER = 2,
}

export interface SensorMsgsJoyFeedbackArray {
  array: SensorMsgsJoyFeedback[];
}

export interface SensorMsgsLaserEcho {
  echoes: number[];
}

export interface SensorMsgsLaserScan {
  header: StdMsgsHeader;
  angle_min: number;
  angle_max: number;
  angle_increment: number;
  time_increment: number;
  scan_time: number;
  range_min: number;
  range_max: number;
  ranges: number[];
  intensities: number[];
}

export interface SensorMsgsMagneticField {
  header: StdMsgsHeader;
  magnetic_field: GeometryMsgsVector3;
  magnetic_field_covariance: number[];
}

export interface SensorMsgsMultiDofJointState {
  header: StdMsgsHeader;
  joint_names: string[];
  transforms: GeometryMsgsTransform[];
  twist: GeometryMsgsTwist[];
  wrench: GeometryMsgsWrench[];
}

export interface SensorMsgsMultiEchoLaserScan {
  header: StdMsgsHeader;
  angle_min: number;
  angle_max: number;
  angle_increment: number;
  time_increment: number;
  scan_time: number;
  range_min: number;
  range_max: number;
  ranges: SensorMsgsLaserEcho[];
  intensities: SensorMsgsLaserEcho[];
}

export interface SensorMsgsNavSatFix {
  header: StdMsgsHeader;
  status: SensorMsgsNavSatStatus;
  latitude: number;
  longitude: number;
  altitude: number;
  position_covariance: number[];
  position_covariance_type: number;
}

export enum SensorMsgsNavSatFixConst {
  COVARIANCE_TYPE_UNKNOWN = 0,
  COVARIANCE_TYPE_APPROXIMATED = 1,
  COVARIANCE_TYPE_DIAGONAL_KNOWN = 2,
  COVARIANCE_TYPE_KNOWN = 3,
}

export interface SensorMsgsNavSatStatus {
  status: number;
  service: number;
}

export enum SensorMsgsNavSatStatusConst {
  STATUS_NO_FIX = -1,
  STATUS_FIX = 0,
  STATUS_SBAS_FIX = 1,
  STATUS_GBAS_FIX = 2,
  SERVICE_GPS = 1,
  SERVICE_GLONASS = 2,
  SERVICE_COMPASS = 4,
  SERVICE_GALILEO = 8,
}

export interface SensorMsgsPointCloud {
  header: StdMsgsHeader;
  points: GeometryMsgsPoint32[];
  channels: SensorMsgsChannelFloat32[];
}

export interface SensorMsgsPointCloud2 {
  header: StdMsgsHeader;
  height: number;
  width: number;
  fields: SensorMsgsPointField[];
  is_bigendian: boolean;
  point_step: number;
  row_step: number;
  data: number[];
  is_dense: boolean;
}

export interface SensorMsgsPointField {
  name: string;
  offset: number;
  datatype: number;
  count: number;
}

export enum SensorMsgsPointFieldConst {
  INT8 = 1,
  UINT8 = 2,
  INT16 = 3,
  UINT16 = 4,
  INT32 = 5,
  UINT32 = 6,
  FLOAT32 = 7,
  FLOAT64 = 8,
}

export interface SensorMsgsRange {
  header: StdMsgsHeader;
  radiation_type: number;
  field_of_view: number;
  min_range: number;
  max_range: number;
  range: number;
}

export enum SensorMsgsRangeConst {
  ULTRASOUND = 0,
  INFRARED = 1,
}

export interface SensorMsgsRegionOfInterest {
  x_offset: number;
  y_offset: number;
  height: number;
  width: number;
  do_rectify: boolean;
}

export interface SensorMsgsRelativeHumidity {
  header: StdMsgsHeader;
  relative_humidity: number;
  variance: number;
}

export interface SensorMsgsSetCameraInfoRequest {
  camera_info: SensorMsgsCameraInfo;
}

export interface SensorMsgsSetCameraInfoResponse {
  success: boolean;
  status_message: string;
}

export interface SensorMsgsTemperature {
  header: StdMsgsHeader;
  temperature: number;
  variance: number;
}

export interface SensorMsgsTimeReference {
  header: StdMsgsHeader;
  time_ref: { secs: number, nsecs: number };
  source: string;
}

export interface ShapeMsgsMesh {
  triangles: ShapeMsgsMeshTriangle[];
  vertices: GeometryMsgsPoint[];
}

export interface ShapeMsgsMeshTriangle {
  vertex_indices: number[];
}

export interface ShapeMsgsPlane {
  coef: number[];
}

export interface ShapeMsgsSolidPrimitive {
  type: number;
  dimensions: number[];
}

export enum ShapeMsgsSolidPrimitiveConst {
  BOX = 1,
  SPHERE = 2,
  CYLINDER = 3,
  CONE = 4,
  BOX_X = 0,
  BOX_Y = 1,
  BOX_Z = 2,
  SPHERE_RADIUS = 0,
  CYLINDER_HEIGHT = 0,
  CYLINDER_RADIUS = 1,
  CONE_HEIGHT = 0,
  CONE_RADIUS = 1,
}

export interface SmachMsgsSmachContainerInitialStatusCmd {
  path: string;
  initial_states: string[];
  local_data: string;
}

export interface SmachMsgsSmachContainerStatus {
  header: StdMsgsHeader;
  path: string;
  initial_states: string[];
  active_states: string[];
  local_data: string;
  info: string;
}

export interface SmachMsgsSmachContainerStructure {
  header: StdMsgsHeader;
  path: string;
  children: string[];
  internal_outcomes: string[];
  outcomes_from: string[];
  outcomes_to: string[];
  container_outcomes: string[];
}

export interface StdMsgsBool {
  data: boolean;
}

export interface StdMsgsByte {
  data: number;
}

export interface StdMsgsByteMultiArray {
  layout: StdMsgsMultiArrayLayout;
  data: number[];
}

export interface StdMsgsChar {
  data: number;
}

export interface StdMsgsColorRgba {
  r: number;
  g: number;
  b: number;
  a: number;
}

export interface StdMsgsDuration {
  data: { secs: number, nsecs: number };
}

export interface StdMsgsFloat32 {
  data: number;
}

export interface StdMsgsFloat32MultiArray {
  layout: StdMsgsMultiArrayLayout;
  data: number[];
}

export interface StdMsgsFloat64 {
  data: number;
}

export interface StdMsgsFloat64MultiArray {
  layout: StdMsgsMultiArrayLayout;
  data: number[];
}

export interface StdMsgsHeader {
  seq: number;
  stamp: { secs: number, nsecs: number };
  frame_id: string;
}

export interface StdMsgsInt16 {
  data: number;
}

export interface StdMsgsInt16MultiArray {
  layout: StdMsgsMultiArrayLayout;
  data: number[];
}

export interface StdMsgsInt32 {
  data: number;
}

export interface StdMsgsInt32MultiArray {
  layout: StdMsgsMultiArrayLayout;
  data: number[];
}

export interface StdMsgsInt64 {
  data: number;
}

export interface StdMsgsInt64MultiArray {
  layout: StdMsgsMultiArrayLayout;
  data: number[];
}

export interface StdMsgsInt8 {
  data: number;
}

export interface StdMsgsInt8MultiArray {
  layout: StdMsgsMultiArrayLayout;
  data: number[];
}

export interface StdMsgsMultiArrayDimension {
  label: string;
  size: number;
  stride: number;
}

export interface StdMsgsMultiArrayLayout {
  dim: StdMsgsMultiArrayDimension[];
  data_offset: number;
}

export interface StdMsgsString {
  data: string;
}

export interface StdMsgsTime {
  data: { secs: number, nsecs: number };
}

export interface StdMsgsUInt16 {
  data: number;
}

export interface StdMsgsUInt16MultiArray {
  layout: StdMsgsMultiArrayLayout;
  data: number[];
}

export interface StdMsgsUInt32 {
  data: number;
}

export interface StdMsgsUInt32MultiArray {
  layout: StdMsgsMultiArrayLayout;
  data: number[];
}

export interface StdMsgsUInt64 {
  data: number;
}

export interface StdMsgsUInt64MultiArray {
  layout: StdMsgsMultiArrayLayout;
  data: number[];
}

export interface StdMsgsUInt8 {
  data: number;
}

export interface StdMsgsUInt8MultiArray {
  layout: StdMsgsMultiArrayLayout;
  data: number[];
}

export interface StdSrvsSetBoolRequest {
  data: boolean;
}

export interface StdSrvsSetBoolResponse {
  success: boolean;
  message: string;
}

export interface StdSrvsTriggerResponse {
  success: boolean;
  message: string;
}

export interface StereoMsgsDisparityImage {
  header: StdMsgsHeader;
  image: SensorMsgsImage;
  f: number;
  T: number;
  valid_window: SensorMsgsRegionOfInterest;
  min_disparity: number;
  max_disparity: number;
  delta_d: number;
}

export interface Tf2MsgsFrameGraphResponse {
  frame_yaml: string;
}

export interface Tf2MsgsLookupTransformAction {
  action_goal: Tf2MsgsLookupTransformActionGoal;
  action_result: Tf2MsgsLookupTransformActionResult;
  action_feedback: Tf2MsgsLookupTransformActionFeedback;
}

export interface Tf2MsgsLookupTransformActionFeedback {
  header: StdMsgsHeader;
  status: ActionlibMsgsGoalStatus;
}

export interface Tf2MsgsLookupTransformActionGoal {
  header: StdMsgsHeader;
  goal_id: ActionlibMsgsGoalId;
  goal: Tf2MsgsLookupTransformGoal;
}

export interface Tf2MsgsLookupTransformActionGoal {
  target_frame: string;
  source_frame: string;
  source_time: { secs: number, nsecs: number };
  timeout: { secs: number, nsecs: number };
  target_time: { secs: number, nsecs: number };
  fixed_frame: string;
  advanced: boolean;
}

export interface Tf2MsgsLookupTransformActionResult {
  header: StdMsgsHeader;
  status: ActionlibMsgsGoalStatus;
  result: Tf2MsgsLookupTransformResult;
}

export interface Tf2MsgsLookupTransformActionResult {
  transform: GeometryMsgsTransformStamped;
  error: Tf2MsgsTf2Error;
}

export interface Tf2MsgsLookupTransformGoal {
  target_frame: string;
  source_frame: string;
  source_time: { secs: number, nsecs: number };
  timeout: { secs: number, nsecs: number };
  target_time: { secs: number, nsecs: number };
  fixed_frame: string;
  advanced: boolean;
}

export interface Tf2MsgsLookupTransformResult {
  transform: GeometryMsgsTransformStamped;
  error: Tf2MsgsTf2Error;
}

export interface Tf2MsgsTf2Error {
  error: number;
  error_string: string;
}

export enum Tf2MsgsTf2ErrorConst {
  NO_ERROR = 0,
  LOOKUP_ERROR = 1,
  CONNECTIVITY_ERROR = 2,
  EXTRAPOLATION_ERROR = 3,
  INVALID_ARGUMENT_ERROR = 4,
  TIMEOUT_ERROR = 5,
  TRANSFORM_ERROR = 6,
}

export interface Tf2MsgsTfMessage {
  transforms: GeometryMsgsTransformStamped[];
}

export interface TfFrameGraphResponse {
  dot_graph: string;
}

export interface TfTfMessage {
  transforms: GeometryMsgsTransformStamped[];
}

export interface TheoraImageTransportPacket {
  header: StdMsgsHeader;
  data: number[];
  b_o_s: number;
  e_o_s: number;
  granulepos: number;
  packetno: number;
}

export interface TopicToolsDemuxAddRequest {
  topic: string;
}

export interface TopicToolsDemuxDeleteRequest {
  topic: string;
}

export interface TopicToolsDemuxListResponse {
  topics: string[];
}

export interface TopicToolsDemuxSelectRequest {
  topic: string;
}

export interface TopicToolsDemuxSelectResponse {
  prev_topic: string;
}

export interface TopicToolsMuxAddRequest {
  topic: string;
}

export interface TopicToolsMuxDeleteRequest {
  topic: string;
}

export interface TopicToolsMuxListResponse {
  topics: string[];
}

export interface TopicToolsMuxSelectRequest {
  topic: string;
}

export interface TopicToolsMuxSelectResponse {
  prev_topic: string;
}

export interface TrajectoryMsgsJointTrajectory {
  header: StdMsgsHeader;
  joint_names: string[];
  points: TrajectoryMsgsJointTrajectoryPoint[];
}

export interface TrajectoryMsgsJointTrajectoryPoint {
  positions: number[];
  velocities: number[];
  accelerations: number[];
  effort: number[];
  time_from_start: { secs: number, nsecs: number };
}

export interface TrajectoryMsgsMultiDofJointTrajectory {
  header: StdMsgsHeader;
  joint_names: string[];
  points: TrajectoryMsgsMultiDofJointTrajectoryPoint[];
}

export interface TrajectoryMsgsMultiDofJointTrajectoryPoint {
  transforms: GeometryMsgsTransform[];
  velocities: GeometryMsgsTwist[];
  accelerations: GeometryMsgsTwist[];
  time_from_start: { secs: number, nsecs: number };
}

export interface UuidMsgsUniqueId {
  uuid: number[];
}

export interface VisualizationMsgsImageMarker {
  header: StdMsgsHeader;
  ns: string;
  id: number;
  type: number;
  action: number;
  position: GeometryMsgsPoint;
  scale: number;
  outline_color: StdMsgsColorRgba;
  filled: number;
  fill_color: StdMsgsColorRgba;
  lifetime: { secs: number, nsecs: number };
  points: GeometryMsgsPoint[];
  outline_colors: StdMsgsColorRgba[];
}

export enum VisualizationMsgsImageMarkerConst {
  CIRCLE = 0,
  LINE_STRIP = 1,
  LINE_LIST = 2,
  POLYGON = 3,
  POINTS = 4,
  ADD = 0,
  REMOVE = 1,
}

export interface VisualizationMsgsInteractiveMarker {
  header: StdMsgsHeader;
  pose: GeometryMsgsPose;
  name: string;
  description: string;
  scale: number;
  menu_entries: VisualizationMsgsMenuEntry[];
  controls: VisualizationMsgsInteractiveMarkerControl[];
}

export interface VisualizationMsgsInteractiveMarkerControl {
  name: string;
  orientation: GeometryMsgsQuaternion;
  orientation_mode: number;
  interaction_mode: number;
  always_visible: boolean;
  markers: VisualizationMsgsMarker[];
  independent_marker_orientation: boolean;
  description: string;
}

export enum VisualizationMsgsInteractiveMarkerControlConst {
  INHERIT = 0,
  FIXED = 1,
  VIEW_FACING = 2,
  NONE = 0,
  MENU = 1,
  BUTTON = 2,
  MOVE_AXIS = 3,
  MOVE_PLANE = 4,
  ROTATE_AXIS = 5,
  MOVE_ROTATE = 6,
  MOVE_3D = 7,
  ROTATE_3D = 8,
  MOVE_ROTATE_3D = 9,
}

export interface VisualizationMsgsInteractiveMarkerFeedback {
  header: StdMsgsHeader;
  client_id: string;
  marker_name: string;
  control_name: string;
  event_type: number;
  pose: GeometryMsgsPose;
  menu_entry_id: number;
  mouse_point: GeometryMsgsPoint;
  mouse_point_valid: boolean;
}

export enum VisualizationMsgsInteractiveMarkerFeedbackConst {
  KEEP_ALIVE = 0,
  POSE_UPDATE = 1,
  MENU_SELECT = 2,
  BUTTON_CLICK = 3,
  MOUSE_DOWN = 4,
  MOUSE_UP = 5,
}

export interface VisualizationMsgsInteractiveMarkerInit {
  server_id: string;
  seq_num: number;
  markers: VisualizationMsgsInteractiveMarker[];
}

export interface VisualizationMsgsInteractiveMarkerPose {
  header: StdMsgsHeader;
  pose: GeometryMsgsPose;
  name: string;
}

export interface VisualizationMsgsInteractiveMarkerUpdate {
  server_id: string;
  seq_num: number;
  type: number;
  markers: VisualizationMsgsInteractiveMarker[];
  poses: VisualizationMsgsInteractiveMarkerPose[];
  erases: string[];
}

export enum VisualizationMsgsInteractiveMarkerUpdateConst {
  KEEP_ALIVE = 0,
  UPDATE = 1,
}

export interface VisualizationMsgsMarker {
  header: StdMsgsHeader;
  ns: string;
  id: number;
  type: number;
  action: number;
  pose: GeometryMsgsPose;
  scale: GeometryMsgsVector3;
  color: StdMsgsColorRgba;
  lifetime: { secs: number, nsecs: number };
  frame_locked: boolean;
  points: GeometryMsgsPoint[];
  colors: StdMsgsColorRgba[];
  text: string;
  mesh_resource: string;
  mesh_use_embedded_materials: boolean;
}

export enum VisualizationMsgsMarkerConst {
  ARROW = 0,
  CUBE = 1,
  SPHERE = 2,
  CYLINDER = 3,
  LINE_STRIP = 4,
  LINE_LIST = 5,
  CUBE_LIST = 6,
  SPHERE_LIST = 7,
  POINTS = 8,
  TEXT_VIEW_FACING = 9,
  MESH_RESOURCE = 10,
  TRIANGLE_LIST = 11,
  ADD = 0,
  MODIFY = 0,
  DELETE = 2,
  DELETEALL = 3,
}

export interface VisualizationMsgsMarkerArray {
  markers: VisualizationMsgsMarker[];
}

export interface VisualizationMsgsMenuEntry {
  id: number;
  parent_id: number;
  title: string;
  command: string;
  command_type: number;
}

export enum VisualizationMsgsMenuEntryConst {
  FEEDBACK = 0,
  ROSRUN = 1,
  ROSLAUNCH = 2,
}