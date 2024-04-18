# Duke Robotics Club - Foxglove Extensions & Layouts

Duke Robotics uses [Foxglove](https://foxglove.dev/studio) as its data visualization and controls platform.
This directory contains Duke Robotics extensions and layouts.

## Setup
### Dependencies
- [npm (Latest)](https://docs.npmjs.com/downloading-and-installing-node-js-and-npm)
- [Foxglove Studio (Latest)](https://foxglove.dev/download)
  - **Note:** Automatic installation of custom extensions and layouts requires Foxglove Studio Desktop. For Foxglove Studio Web, layouts must be manually uploaded and custom extensions are not compatible.

### Installation
To manage local installation of Duke Robotics extensions and layouts, use the `foxglove.py` CLI.
To install all Duke Robotics extensions and layouts, use:
```bash
python foxglove.py install
```
Note that installation may take several minutes.

To confirm installation, reload Foxglove (Command/Control+R) and check the `Add panel` button in the top left.
Custom extensions will have the suffix `[local]`. Also, check whether custom layouts have been installed using
the `Layout` button in the top right.

To uninstall all Duke Robotics extensions and layouts, use:
```bash
python foxglove.py uninstall
```

To install only a selection of extensions, use:
```bash
python foxglove.py install -e <extension-1> <extension-2> ...
```
By default, the `-e` flag without arguments will install all extensions.

To uninstall all extensions, use:
```bash
python foxglove.py uninstall -e
```

To install all layouts, use:
```bash
python foxglove.py install -l
```
To uninstall all layouts, use:
```bash
python foxglove.py uninstall -l
```

For more information, consult the usage guide of `foxglove.py` with the `-h` flag:
```bash
python foxglove.py -h
python foxglove.py install -h
python foxglove.py uninstall -h
```

### Development
Ensure that the extension has been installed before starting development.

To test changes made during development, rebuild the extension with:
```bash
npm run local-install
```

For automatic building, run:
```bash
npm run watch:local-install
```
This will automatically execute `npm run local-install` upon `.ts` and `.tsx` file changes in the `src` directory.

### Testing
1. On `onboard`, launch the [Foxglove WebSocket](https://github.com/foxglove/ros-foxglove-bridge) bridge node:
```bash
roslaunch --screen foxglove_bridge foxglove_bridge.launch port:=8765
```
This node enables communication between the `onboard` container and the external Foxglove GUI.

2. In Foxglove, open a `Foxglove WebSocket` connection using the appropriate URL with the hostname or IP of the onboard container:
```
ws://localhost:8765     # Local container
ws://192.168.1.1:8765   # Robot (Tethered)
ws://<hostname or IP of onboard container>:8765
```

### Cleanup
To remove all git ignored files and directories in the foxglove monorepo, run:
```
python foxglove.py clean
```

## Extensions & Layouts
### Panels
- `call-service-panel`: Example panel to call services
- `publish-topic-panel`: Example panel to publish topics
- `subscribe-topic-panel`: Example panel to subscribe to topics and see the raw message feed
- `toggle-controls-panel`: Panel to toggle controls on/off
- `system-status-panel`: Panel that displays system usage of the onboard computer
- `sensors-status-panel`: Panel that displays the connected/disconnected status of Oogway's sensors
- `thruster-allocs-panel`: Panel that displays the current thruster allocs and publishes desired thruster allocs
- `toggle-joystick-panel`: Panel to toggle joystick control on/off, as well as publish transformed joystick inputs as a desired power
- `pid-panel`: Panel to read/set PID gains

### Layouts
- `controls-power.json`: Plots the set power scaled, actual power, and their difference (power disparity) for each axis. Also plots the norm of the power disparity.
- `drc.json`: All Duke Robotics's custom Foxglove panels
- `pid-<position/velocity>.json`: Plots error, control effort, filtered error, integral, filtered derivative, calculated derivative, and provided derivative for each axis
- `sensors.json`: Displays the sensor status, system status, and thruster allocs panels, along with the raw messages from each sensor and all camera feeds.
- `state.json`: Plots state pose, state velocity, desired pose, and desired velocity for all axes, along with IMU orientation (angular axes only) and depth (Z only). It uses the following transformed topics:
  - `/state.pose.pose` transformed to `/transforms/state/pose`
  - `/vectornav/IMU.orientation` transformed to `/transforms/vectornav/IMU/orientation`
  - `/controls/desired_position` transformed to `/transforms/controls/desired_position`

## Local Dependencies
Local dependencies are located in the `shared` directory.
To install all local dependencies, run:
```bash
python foxglove.py build
```

Note that running `python foxglove.py install` will automatically build all local dependencies before installing extensions.

- `theme`: Exports the Duke Robotics MUI Theme
- `defs`: Exports Foxglove datatype maps and TypeScript interfaces/enums for both ROS 1 and Duke Robotics custom message definitions
- `ros-typescript-generator`: CLI that generates TypeScript interfaces/enums from ROS definitions

## Patches
Patches to external node modules are located in the `patches` directory.

Running either `python foxglove.py <build/install>` will automatically install these patches.

- `create-foxglove-extension+0.8.6.patch`
  - No longer require `README.md` or `CHANGELOG.md` when installing an extension
  - Before installing an extension, only remove `dist/extension.js` (instead of cleaning the entire `dist` directory)

## Other Files
- `.eslintrc.json`: Configuration file for ESLint
- `.prettierrc.yaml`: Configuration file for Prettier
- `tsconfig.json`: Configuration file for TypeScript
- `package.json`: Configuration file for npm
- `package-lock.json`: The exact npm dependency tree of the foxglove monorepo, generated using `npm i`
- `empty-layout.json`: Default Foxglove layout JSON file used when creating new layouts from the `foxglove.py` CLI

## Contributing
### Creating a New Extension
Fork an existing Duke Robotics example extension (`call-service-panel`, `publish-topic-panel`, or `subscribe-topic-panel`) to base the new extension off of. This ensures that all of our extensions have the same code structure and use the same core set of dependencies.

### Creating a New Layout
Foxglove does not allow creating more than one layout when not signed in.
To circumvent this issue, use the `foxglove.py` CLI:
```bash
python foxglove.py --new-layout <layout-name>
```
If `<layout-name>` is not specified, the layout name will be the current time in nanoseconds.

Follow the [documentation](https://foxglove.dev/docs/studio/layouts#personal-layouts) to export your layout as a JSON file to the robosub-ros `foxglove/layouts` directory. Manually look over the JSON to ensure that the settings are correct. For example, ensure that `splitPercentage` for each panel is set to the desired amount.

### Creating a New Local Dependency
Fork an existing local dependency (e.g., `theme`). All local dependencies must use TypeScript and an `npm run build` command must be defined in `package.json` so that `foxglove.py` can automatically compile each local dependency to `node_modules`.

## Additional Documentation
- [ExtensionContext](https://docs.foxglove.dev/docs/visualization/extensions/api/extension-context/): Register custom extensions for use in Foxglove
- [PanelExtensionContext](https://docs.foxglove.dev/docs/visualization/extensions/api/panel-api/): API for custom Foxglove panels
- [Panel settings API](https://docs.foxglove.dev/docs/visualization/extensions/api/panel-settings-api/): API to build a settings interface for extension panels
- [Layouts](https://docs.foxglove.dev/docs/visualization/layouts/): Documentation for Foxglove layouts
