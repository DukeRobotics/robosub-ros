# Ros Typescript Generator

[![CircleCI](https://circleci.com/gh/Greenroom-Robotics/ros-typescript-generator/tree/master.svg?style=svg)](https://circleci.com/gh/Greenroom-Robotics/ros-typescript-generator/tree/master)

A CLI for generating typescript interfaces and enums from ros `msg` files. 

### Features

- [x] ROS1 and ROS2 support
- [x] Generate ts types from ROS `msgs`
- [x] Generate ts enums from ROS `msgs`
- [x] Configurable type prefix, eg) `IRosType` or `Ros`
- [x] Works with custom `.msg`, `.srv` and `.action` files 
- [x] No runtime dependencies
- [ ] Advanced multi-enum support
```msg
# example_msgs/example.msg
uint8 STATUS_DISABLED = 0 
uint8 STATUS_ENABLED = 1

uint8 OTHER_THING_1 = 1
uint8 OTHER_THING_2 = 2

uint8 status
uint8 other
uint8 more
```

Should become:
```ts
export enum IRosTypeExampleStatus {
  "STATUS_DISABLED" = 0,
  "STATUS_ENABLED" = 1,
}
export enum IRosTypeExampleOther {
  "OTHER_THING_1" = 1,
  "OTHER_THING_2" = 2
}
export interface IRosTypeExampleOther {
  status: IRosTypeExampleStatus
  other: IRosTypeExampleOther
  more: number
}
```
### Comparison to rclnodejs / rostsd-gen

Unlike [rostsd-gen](https://github.com/RobotWebTools/rclnodejs/tree/develop/rostsd_gen), this **ONLY** generates ts types and enums. This means the output does not include any nodejs dependencies. In fact, it has no runtime dependencies at all. It uses interfaces rather than classes 🙂. This makes it good option for any frontend project.

### Usage

- Add a `ros-ts-generator-config.json` file to your project root. For example:

```json
{
  "output": "./generated/ros_msgs.ts",
  "rosVersion": 2, // 1 or 2
  "input": [
    {
      "namespace": "std_msgs",
      "path": "/opt/ros/iron/share/std_msgs"
    },
    {
      "namespace": "geometry_msgs",
      "path": "/opt/ros/iron/share/geometry_msgs"
    },
    // Add any other messages including your own custom messages.
  ],
  "typePrefix": "IRosType"
}
```
- Run `npx ros-typescript-generator --config ros-ts-generator-config.json`
- Done!

### Examples

* [Basic Example](./examples/basic)
* [Real ROS1 Example](./examples/real-ros1)
* [Real ROS2 Example](./examples/real-ros2)

### Credit

Credit goes to [foxglove](https://github.com/foxglove) for their [foxglove/rosmsg](https://github.com/foxglove/rosmsg) library.