#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float64


class PressureToDepthConverter(Node):

    NODE_NAME = "depth_pub"
    PRESSURE_SUB_TOPIC = "offboard/pressure"
    DEPTH_DEST_TOPIC = "sensors/depth"

    DENSITY_WATER = 1000
    ACCEL_GRAVITY = 9.80665
    ATMOSPHERIC_PRESSURE = 101325

    def __init__(self):
        super().__init__(self.NODE_NAME)
        self._pub_depth = self.create_publisher(Float64, self.DEPTH_DEST_TOPIC, 50)
        self.create_subscription(FluidPressure, self.PRESSURE_SUB_TOPIC, self._receive_pressure, 50)

    def _receive_pressure(self, pressure):
        depth = Float64()
        depth.data = self._pressure_to_depth(pressure.fluid_pressure)
        self._pub_depth.publish(depth)

    def _pressure_to_depth(self, pressure):
        return -(pressure - self.ATMOSPHERIC_PRESSURE) / (self.DENSITY_WATER * self.ACCEL_GRAVITY)


def main(args=None):
    try:
        rclpy.init(args=args)
        converter = PressureToDepthConverter()
        rclpy.spin(converter)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        sys.exit(1)
    finally:
        converter.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
