# Duke Robotics Foxglove Extensions & Layouts

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
**Note:** The user must be signed out of Foxglove to uninstall layouts using this CLI. You can still uninstall layouts manually through the app.

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

## Extensions & Layouts
### Panels
- `call-service-panel`: Example panel to call services
- `publish-topic-panel`: Example panel to publish topics
- `subscribe-topic-panel`: Example panel to subscribe to topics and see the raw message feed
- `toggle-controls-panel`: Panel to toggle controls on/off
- `system-status-panel`: Panel that displays system usage of the onboard computer
- `sensors-status-panel`: Panel that displays the connected/disconnected status of Oogway's sensors


### Layouts
- `controls-monitor.json`: 6 graphs for each control axis (x, y, z, roll, pitch, yaw) plotting setpoint and control effort against time. Used to test responsiveness of robot during PID tuning.

## Local Dependencies
Shared components and styles are located in the `shared` directory.

- `theme`: Exports the Duke Robotics MUI Theme
- `defs`: Exports Foxglove datatype maps and TypeScript interfaces/enums for both ROS 1 and Duke Robotics custom definitions
- `ros-typescript-generator`: CLI that generates TypeScript interfaces/enums from ROS definitions

## Contributing
### Creating a New Extension
Fork an existing Duke Robotics example extension (`call-service-panel`, `publish-topic-panel`, or `subscribe-topic-panel`) to base the new extension off of. This ensures that all of our extensions have the same code structure and use the same core set of dependencies.

### Creating a New Layout
Follow the [documentation](https://foxglove.dev/docs/studio/layouts#personal-layouts) to export your layout as a JSON file to the robosub-ros `foxglove/layouts` directory. Manually look over the JSON to ensure that the settings are correct. For example, ensure that `splitPercentage` for each panel is set to the desired amount.

### Creating a New Local Dependency
Fork an existing local dependency (e.g., `theme`). All local dependencies must use TypeScript and an `npm run build` command must be defined in `package.json` so that `foxglove.py` can automatically compile each local dependency to `node_modules`.

## Additional Documentation
- [ExtensionContext](https://foxglove.dev/docs/studio/extensions/extension-context): Register custom extensions for use in Foxglove
- [PanelExtensionContext](https://foxglove.dev/docs/studio/extensions/panel-api): API for custom Foxglove panels
- [Panel settings API](https://foxglove.dev/docs/studio/extensions/panel-settings-api): API to build a settings interface for extension panels
- [Layouts](https://foxglove.dev/docs/studio/layouts): Documentation for Foxglove layouts