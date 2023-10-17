# Custom Foxglove Studio Extensions & Layouts

Duke Robotics uses [Foxglove](https://foxglove.dev/studio) as its data visualization and controls platform.
This directory contains our custom extensions and layouts.

## Dependencies
- [npm](https://docs.npmjs.com/downloading-and-installing-node-js-and-npm)
- [Yarn](https://classic.yarnpkg.com/lang/en/docs/install/#mac-stable)
- [Foxglove Studio](https://foxglove.dev/download) 

## Installation
To manage local installation of our custom extensions and layouts, use the `foxglove.py` CLI.
To install all Duke Robotics extensions and layouts, use:
```bash
python foxglove.py install
```
Note that installation may take several minutes.

To confirm installation, reload Foxglove (Command/Control+R) and check the `Add panel` button in the top left.
Custom extensions will have the suffix `[local]`.

To uninstall all Duke Robotics extensions and layouts, use:
```bash
python foxglove.py uninstall
```

To install only a selection of extensions, use:
```bash
python foxglove.py install -e <extension-1> <extension-2> ...
```

For more information, consult the usage guide of `foxglove.py` with the `-h` flag:
```bash
python foxglove.py -h
python foxglove.py install -h
python foxglove.py uninstall -h
```

## Development
After the extension is installed, you only need to build the extension with:
```bash
npm run local-install
```

For automatic building, run:
```bash
npm run watch:local-install
```
This will automatically execute `npm run local-install` upon `.ts` and `.tsx` file changes in the `src` directory.

## Testing
1. On `onboard`, launch the Foxglove WebSocket bridge node:
```bash
roslaunch --screen foxglove_bridge foxglove_bridge.launch port:=8765
```

2. In Foxglove, open a `Foxglove WebSocket` connection using the appropriate URL:
```
ws://localhost:8765     # Local container
ws://192.168.1.1:8765   # Robot
```

## Extensions & Layouts
### Panels
- `call-service-panel`: Example panel that lets you call services
- `publish-topic-panel`: Example panel that lets you publish topics
- `subscribe-topic-panel`: Example panel that lets you subscribe to topics and see the raw message feed

### Layouts
- `controls-monitor.json`: 6 graphs for each control axis (x, y, z, roll, pitch, yaw) plotting setpoint and control effort against time. Used to test responsiveness of robot during PID tuning.
