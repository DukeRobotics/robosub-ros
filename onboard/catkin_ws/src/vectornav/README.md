# Vectornav

This package reads data from the VectorNav VN-100 IMU and publishes it to ROS topics. The package was forked from
[dawonn/vectornav](https://github.com/dawonn/vectornav).

The following information is _not_ comprehensive. It only provides an overview of the parts that may need to be modified or are used in the rest of the robot's software.

## Files

- `vectornav.launch`: Launch file for the package. It sets up the serial port and the frame id for the IMU.
- `vn100.yaml`: Configuration file for the VN-100 IMU. It sets up the frame id and the serial port, among other things.
- `99-vn100.rules`: Udev rules file for the VN-100 IMU. It sets up an alias for the device's serial port.
- `LICENSE.txt`: License file for the forked repo.


## Topics

- `/vectornav/IMU`: IMU data from the VN-100 IMU. The message type is `sensor_msgs/Imu`.

## Udev Rules

The VN-100 IMU is connected to the computer via USB. The device's serial port could be any of `/dev/ttyUSB0`, `/dev/ttyUSB1`, etc – it can change each time the computer is started or when the device is unplugged and replugged. However, the `vn100.yaml` config file expects a fixed serial port.

To fix this, a udev rules file is used to create an alias for the device's serial port. The udev rule finds the device's true serial port and symlinks it to a fixed alias. The alias can then be used in the `vn100.yaml` config file.

The alias is `/dev/ttyUSB_VN100`. The udev rules file is `99-vn100.rules`.

> [!IMPORTANT]
> The udev rules are setup on the robot computer directly, _not_ in the Docker container. The container will have the same symlinked alias as the host machine.

The rules file and the following instructions were adapted from [ntnu-arl/vectornav](https://github.com/ntnu-arl/vectornav/tree/main).

### Setting Up Udev Rules
*Perform the followig steps on the robot computer.*
1. Use `udevadm info --attribute-walk /dev/ttyUSB0` (change `/dev/ttyUSB0` to the port that the IMU is connected to) to find the values for your FTDI converter. Typically, `idProduct`, `idVendor` and `serial` should be enough.
2. Edit the `99-vn100.rules` file and replace the values with the ones from the previous step.
3. Edit the `SYMLINK` in the `99-vn100.rules` file and replace the value with the desired name for the serial port alias.
4. Copy the `99-vn100.rules` file to `/etc/udev/rules.d/`.

    ```bash
    cd robosub-ros
    sudo cp udev/99-vn100.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules && udevadm trigger
    ```

5. Add the logged in user to the `dialout` group with `sudo usermod -a -G dialout $USER`.
6. Restart the computer.

Note:

1. The USB latency timer should be set to 1 for the IMU to avoid the bunching up of IMU messages. The udev rule automatically does this for you.
2. You can use `udevadm test $(udevadm info --query=path --name=/dev/ttyUSB0)` to test whether the new rule works as expected. If the rule works, it should show you the attributes for the converter with the rule that was triggered.
3. You may need to unplug and replug the IMU to make the udev rule take effect.
4. If the udev rules do not load, you may need to restart the computer. This only needs to be done once during the setup.
