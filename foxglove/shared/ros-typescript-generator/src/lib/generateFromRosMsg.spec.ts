import test from 'ava';

import { generateFromRosMsg } from './generateFromRosMsg';

test('generateFromRosMsg with comments', (t) => {
  const result = generateFromRosMsg(
    `MSG: geometry_msgs/Point
  # This was originally provided as an example message.
  # It is deprecated as of Foxy
  # It is recommended to create your own semantically meaningful message.
  # However if you would like to continue using this please use the equivalent in example_msgs.

  string label   # label of given dimension
  uint32 size    # size of given dimension (in type units)
  uint32 stride  # stride of given dimension
  `,
    'RosType'
  );
  const expected = `export interface RosTypeGeometryMsgsPoint {
  label: string;
  size: number;
  stride: number;
}`;
  t.is(result, expected);
});

test('generateFromRosMsg multi message', (t) => {
  const result = generateFromRosMsg(
    `MSG: geometry_msgs/Pose
  geometry_msgs/Point position
  geometry_msgs/Quaternion orientation

  ===
  MSG: geometry_msgs/Point
  float64 x
  float64 y
  float64 z

  ===
  MSG: geometry_msgs/Quaternion
  float64 x
  float64 y
  float64 z
  float64 w`,
    'RosType'
  );

  const expected = `export interface RosTypeGeometryMsgsPoint {
  x: number;
  y: number;
  z: number;
}

export interface RosTypeGeometryMsgsPose {
  position: RosTypeGeometryMsgsPoint;
  orientation: RosTypeGeometryMsgsQuaternion;
}

export interface RosTypeGeometryMsgsQuaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}`;
  t.is(result, expected);
});

test('generateFromRosMsg with enum', (t) => {
  const result = generateFromRosMsg(
    `MSG: example/Message
  uint8 NONE=0
  uint8 PARK=1
  uint8 REVERSE=2
  uint8 NEUTRAL=3
  uint8 DRIVE=4
  uint8 LOW=5
  uint8 gear`
  );

  const expected = `export interface ExampleMessage {
  gear: number;
}

export enum ExampleMessageConst {
  NONE = 0,
  PARK = 1,
  REVERSE = 2,
  NEUTRAL = 3,
  DRIVE = 4,
  LOW = 5,
}`;
  t.is(result, expected);
});

test.skip('generateFromRosMsg with enum which are prefixed', (t) => {
  const result = generateFromRosMsg(
    `MSG: example/Message

  uint8 blinker_state
  uint8 headlight_state
  uint8 wiper_state
  uint8 current_gear
  uint8 vehicle_mode
  bool hand_brake_active
  bool horn_active
  bool autonomous_mode_active

  uint8 BLINKERS_OFF = 0
  uint8 BLINKERS_LEFT = 1
  uint8 BLINKERS_RIGHT = 2
  uint8 BLINKERS_HAZARD = 3

  uint8 HEADLIGHTS_OFF = 0
  uint8 HEADLIGHTS_LOW = 1
  uint8 HEADLIGHTS_HIGH = 2

  uint8 WIPERS_OFF = 0
  uint8 WIPERS_LOW = 1
  uint8 WIPERS_MED = 2
  uint8 WIPERS_HIGH = 3

  uint8 GEAR_NEUTRAL = 0
  uint8 GEAR_DRIVE = 1
  uint8 GEAR_REVERSE = 2
  uint8 GEAR_PARKING = 3
  uint8 GEAR_LOW = 4

  uint8 VEHICLE_MODE_COMPLETE_MANUAL = 0
  uint8 VEHICLE_MODE_COMPLETE_AUTO_DRIVE = 1
  uint8 VEHICLE_MODE_AUTO_STEER_ONLY = 2
  uint8 VEHICLE_MODE_AUTO_SPEED_ONLY = 3
  uint8 VEHICLE_MODE_EMERGENCY_MODE = 4`
  );

  const expected = ` `;
  t.is(result, expected);
});

test('generateFromRosMsg with ROS1 msg', (t) => {
  const rosVersion = 1;
  // Message partially from
  // http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html
  const result = generateFromRosMsg(
    `
  float64[] D
  float64[9]  K # 3x3 row-major matrix
  float64[9]  R # 3x3 row-major matrix
  float64[12] P # 3x4 row-major matrix

  uint32 binning_x
  uint32 binning_y
  `,
    '',
    rosVersion
  );

  const expected = `export interface  {
  D: number[];
  K: number[];
  R: number[];
  P: number[];
  binning_x: number;
  binning_y: number;
}`;

  t.is(result, expected);
});

test('generateFromRosMsg with prefix', (t) => {
  const result = generateFromRosMsg(
    `MSG: package/Type
float64 x
float64 y
float64 z
`,
    'Prefix'
  );

  const expected = `export interface PrefixPackageType {
  x: number;
  y: number;
  z: number;
}`;

  t.is(result, expected);
});

test('generateFromRosMsg with empty type', (t) => {
  const result = generateFromRosMsg(
    `MSG: test_msgs/HasOneEmpty
  test_msgs/Empty empty
  test_msgs/Normal normal

  ===
  MSG: test_msgs/Empty

  ===
  MSG: test_msgs/Normal
  float64 x
  float64 y`,
    'Prefix'
  );

  const expected = `export interface PrefixTestMsgsHasOneEmpty {
  normal: PrefixTestMsgsNormal;
}

export interface PrefixTestMsgsNormal {
  x: number;
  y: number;
}`;

  t.is(result, expected);
});

test('generateFromRosMsg with bool enum', (t) => {
  const result = generateFromRosMsg(
    `MSG: test_msgs/State
  bool OFF = 0
  bool ON = 1
`
  );

  const expected = `export enum TestMsgsStateConst {
  OFF = 0,
  ON = 1,
}`;

  t.is(result, expected);
});

test('generateFromRosMsg with string enum', (t) => {
  const result = generateFromRosMsg(
    `MSG: test_msgs/State
  string OFF = 'off'
  string ON = 'on'
`
  );

  const expected = `export enum TestMsgsStateConst {
  OFF = 'off',
  ON = 'on',
}`;

  t.is(result, expected);
});

test('generateFromRosMsg with duration & time (ROS 1)', (t) => {
  const result = generateFromRosMsg(
    `MSG: test_msgs/time
  duration period
  time stamp
`,
    '',
    1
  );

  const expected = `export interface TestMsgsTime {
  period: { secs: number, nsecs: number };
  stamp: { secs: number, nsecs: number };
}`;

  t.is(result, expected);
});

test('generateFromRosMsg with duration & time (ROS 2)', (t) => {
  const result = generateFromRosMsg(
    `MSG: test_msgs/time
  duration period
  time stamp
`,
    '',
    2
  );

  const expected = `export interface TestMsgsTime {
  period: { sec: number, nanosec: number };
  stamp: { sec: number, nanosec: number };
}`;

  t.is(result, expected);
});
