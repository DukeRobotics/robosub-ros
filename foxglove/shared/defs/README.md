# Foxglove Definitions
This package exports datatype maps and TypeScript interfaces/enums for both ROS 1 and Duke Robotics custom definitions.
Datatype maps are needed to advertise topics. TypeScript interfaces are used for static type checking.

## Usage
### Datatype Maps
Foxglove requires a [datatype map](https://docs.foxglove.dev/docs/visualization/extensions/api/panel-api#native-ros-1) 
when advertising a topic in an extension. Instead of manually constructing the `MessageDefinition` definitions or `datatypes`,
this package generates the dataype maps of all ROS 1 and Duke Robotics custom definitions.

Import with
```js
import { allDatatypeMaps } from "@duke-robotics/defs/datatype_maps";
```
Then, advertise using the appropriate map. For example, to advertise thruster speeds, use:
```js
context.advertise(`<Topic Name>`, "custom_msgs/ThrusterSpeeds", {
    datatypes: allDatatypeMaps.custom_msgs["custom_msgs/ThrusterSpeeds"],
});
```

### TypeScript Interfaces/Enums
To enforce static type checking of ROS messages in Foxglove extensions, an interface must be specified
that describes the message's shape. This package exports all ROS 1 and Duke Robotics custom definitions
as TypeScript interfaces. Any constants in the definition are exported as a TypeScript `enum`.

Import with
```js
import { <TypeName> } from "@duke-robotics/defs/types";
```

To view all available types, see `foxglove/shared/defs/types/dist/types.ts`.

Note that the type is prefixed by its package name. For example, `GeometryMsgsTwist` comes from the `geometry_msgs` ROS 1 package.

Additionally, the type is suffixed if it describes a ROS service, action, or constant. For example, the `SetBool` service
has 2 types: `StdSrvsSetBoolRequest` and `StdSrvsSetBoolResponse`.

## Generating ROS 1 Types
The `types/ros1Msgs.ts` file exports the types of all ROS 1 messages, services, and actions. To regenerate this file, follow the following steps:

1. Build necessary dependencies
```bash
cd foxglove
python foxglove build
```
2. Start the onboard docker container
3. Copy over the ROS 1 `share` directory using `scp`
```bash
cd foxglove/shared/defs/
scp -P 2200 -r root@localhost:/opt/ros/noetic/share/ share
```
4. Generate the types
```bash
npx ros-typescript-generator --config ros_ts_generator_configs/ros1.json
```