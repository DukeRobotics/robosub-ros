# Sensors Status Panel
Sensors Status Panel displays the connected/disconnected status of robot sensors. Connected means the relevant sensor topic has published within the last second.

## Usage
Connect to a data source (robot, local container, or bag file). The panel displays a table of sensors. A red background color indicates the sensor is down; a green background color indicates the sensor is active. Hover above the sensor name to examine the exact topic that is being watched.

## Constants
- `TOPICS_MAP`: An object containing sensor name to topic name mappings.
- `SECONDS_SENSOR_DOWN_THRESHOLD`: Seconds until sensor is considered disconnected. This value should be set according to the sensor with the lowest publish rate. For example, if the slowest sensor is expected to publish at 2 Hz, a reasonable value would be 1 second.