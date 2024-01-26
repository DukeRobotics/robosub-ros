# Foxglove Definitions
This package exports datatype maps and TypeScript interfaces/enums for both ROS 1 and Duke Robotics custom message definitions.
Datatype maps are needed to advertise topics. TypeScript interfaces are used for static type checking.

This package is based off of the Foxglove [rosmsg-msgs-common](https://github.com/foxglove/rosmsg-msgs-common) repository.

## Usage
### Datatype Maps
Foxglove requires a [datatype map](https://docs.foxglove.dev/docs/visualization/extensions/api/panel-api#native-ros-1) 
when advertising a topic in an extension. Instead of manually constructing the `MessageDefinition` definitions or `datatypes`,
this package generates the dataype maps of all ROS 1 and Duke Robotics custom definitions.

Import with
```js
import { allDatatypeMaps } from "@duke-robotics/defs/datatype_maps";
```
Then, advertise using the appropriate map. For example, to advertise thruster allocs, use:
```js
context.advertise(`<Topic Name>`, "custom_msgs/ThrusterAllocs", {
    datatypes: allDatatypeMaps.custom_msgs["custom_msgs/ThrusterAllocs"],
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

#### Interface Translation
Interface names are translated from snake_case to PascalCase.
The interface is prefixed by its package name. For example, `GeometryMsgsTwist` comes from the `geometry_msgs` ROS 1 package.

##### Messages
[ROS messages](http://wiki.ros.org/msg) are translated directly to one interface (with no suffix).

##### Services
[ROS services](http://wiki.ros.org/Services) are split into two interfaces.
- The *request* (before the `---`) has the suffix `request`.
- The *reply* (after the `---`) has the suffix `response`.

##### Actions
[ROS actions](http://wiki.ros.org/actionlib) are split into three interfaces.
- The *goal* has the suffix `ActionGoal`
- The *feedback* has the suffix `ActionFeedback`
- The *result* has the suffix `ActionResult`

##### Constants
Constants are translated into an enum with the suffix `Const`.

## Generating ROS 1 Types
The `types/ros1Msgs.ts` file exports the types of all ROS 1 messages, services, and actions. In the rare case that this file needs to be regenerated (e.g., ROS message definitions change, TypeScript type generation algorithm changes), follow the following steps:

**NOTE:** There are some duplicates in the `ros1Msgs.ts` file! (e.g., `ActionlibTestRequestActionGoal`.)
However, all duplicates come from ROS actions and Foxglove does not currently support ROS actions.
Therefore, this conflict can be ignored for now.

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
The `share` folder can now be safely deleted.