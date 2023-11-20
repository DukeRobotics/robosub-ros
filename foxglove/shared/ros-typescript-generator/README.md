# ROS Typescript Generator
A CLI for generating typescript interfaces and enums from ros `.msg`, `.srv`, and `.action` files.

This package has been modified to provide support for the conversion of custom messages without ROS message dependencies. The [original package](https://github.com/Greenroom-Robotics/ros-typescript-generator) was created by Greenroom Robotics.

## Example
This ROS message
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

will become
```ts
export enum TypeExampleStatus {
  "STATUS_DISABLED" = 0,
  "STATUS_ENABLED" = 1,
}
export enum TypeExampleOther {
  "OTHER_THING_1" = 1,
  "OTHER_THING_2" = 2
}
export interface TypeExampleOther {
  status: TypeExampleStatus
  other: TypeExampleOther
  more: number
}
```
### Usage

1. Add a `ros-ts-generator-config.json` file to your project root. For example:

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
2. Run `npx ros-typescript-generator --config ros-ts-generator-config.json`