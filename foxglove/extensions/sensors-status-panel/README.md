# Sensors Status Panel
This is a panel that displays the connected/disconnected status of sensors. A sensor is connected when the relevant sensor topic has published within the last second.

## Usage
The panel displays a table of sensors that are being watched. A red background color indicates the sensor is down; a green background color indicates the sensor is connected. Hover above the sensor name to examine the exact topic that is being watched.

## Configuration
The following constants can be modified:
- `TOPICS_MAP`: An object containing sensor name to topic name mappings. To watch a new sensor, add a new key-value pair to this object.
- `SENSOR_DOWN_THRESHOLD`: Seconds until sensor is considered disconnected. This value should be set according to the sensor with the lowest publish rate. For example, if the slowest sensor is expected to publish at 2 Hz, a reasonable value would be 1 second.