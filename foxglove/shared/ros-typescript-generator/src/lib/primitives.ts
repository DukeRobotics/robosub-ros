/**
 * Common types between ROS 1 & 2
 */
export const common = {
  bool: 'boolean',
  byte: 'Int8Array',
  char: 'string',
  float32: 'number',
  float64: 'number',
  int8: 'number',
  uint8: 'number',
  int16: 'number',
  uint16: 'number',
  int32: 'number',
  uint32: 'number',
  int64: 'number',
  uint64: 'number',
  string: 'string',
  wstring: 'string',
  wchar: 'string',
};

/**
 * ROS 1
 */
export const primitives1 = {
  ...common,
  duration: '{ secs: number, nsecs: number }',
  time: '{ secs: number, nsecs: number }',
};

/**
 * Map all ROS 2 primitives to a ts type
 * https://design.ros2.org/articles/interface_definition.html
 */
export const primitives2 = {
  ...common,
  duration: '{ sec: number, nanosec: number }',
  time: '{ sec: number, nanosec: number }',
};
