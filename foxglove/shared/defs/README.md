# Theme
Description

## Usage
Import with
```js
```

## Generating ROS 1 Types
The `types/ros1Msgs.ts` file exports the types of all ROS 1 messages, services, and actions. To regenerate this file, follow the following steps:

1. Build necessary dependencies
```bash
cd robosub-ros/foxglove
python foxglove build
```
2. Start the onboard docker container
3. Copy over the ROS 1 shared directory using `scp`
```bash
cd robosub-ros/foxglove/shared/defs/
scp -P 2200 -r root@localhost:/opt/ros/noetic/share/ share
```
4. Generate the types
```bash
npx ros-typescript-generator --config ros_typescript_generator_configs/ros1.json
```