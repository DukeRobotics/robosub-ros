# ROS TypeScript Generator
A CLI for generating typescript interfaces and enums from ros `.msg`, `.srv`, and `.action` files.

This package has been modified to provide support for the conversion of custom messages without ROS message dependencies. Note that `@foxglove/rosmsg` has been updated from `^1.0.1` to `^5.0.2` in order to make the necessary modifications. The [original package](https://github.com/Greenroom-Robotics/ros-typescript-generator) was created by Greenroom Robotics.

## Example
This `RemoteLaunchInfo.msg`
```msg
uint32 msg_type

uint32 EXECUTING = 0
uint32 TERMINATING = 1

custom_msgs/RunningNode running_node_info
```

will become
```ts
export interface CustomMsgsRemoteLaunchInfo {
  msg_type: number;
  running_node_info: CustomMsgsRunningNode;
}
export enum CustomMsgsRemoteLaunchInfoConst {
  EXECUTING = 0,
  TERMINATING = 1,
}

```
## Usage

1. Create a `ros-ts-generator-config.json` file. See `foxglove/shared/defs/ros_ts_generator_configs` for examples.
2. Run `npx ros-typescript-generator --config ros-ts-generator-config.json` to generate the TypeScript types.

## Additional Documentation
See the [original repositiory](https://github.com/Greenroom-Robotics/ros-typescript-generator) for additional documentation
and examples.