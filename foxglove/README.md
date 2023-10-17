# Duke Robotics Club - Foxglove Studio Extensions

Duke Robotics uses [Foxglove](https://foxglove.dev/studio) as its data visualization and controls platform.
This directory contains our custom extensions and layouts. See the wiki for more details.

## Dependencies
- [npm](https://docs.npmjs.com/downloading-and-installing-node-js-and-npm)
- yarn: `npm -g install yarn`
- [Foxglove Studio](https://foxglove.dev/download) 

## Installation
To manage local installation of our custom Foxglove extensions and layouts, use the `foxglove.py` CLI.
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

## Development
After the extension is installed, you only need to build the extension using:
```bash
npm run local-install
```

For automatic building, run:
```bash
npm run watch:local-install
```
This will automatically run `npm run local-install` upon file changes in the `src` directory.

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

## Panels
- `call-service-panel`: Example panel that lets you call services
- `publish-topic-panel`: Example panel that lets you publish topics
- `subscribe-topic-panel`: Example panel that lets you subscribe to topics and see the raw message feed


## Layouts
- `controls-monitor.json`: 6 graphs for each control axis (x, y, z, roll, pitch, yaw) plotting setpoint and control effort against time. Used to test responsiveness of robot during PID tuning.
