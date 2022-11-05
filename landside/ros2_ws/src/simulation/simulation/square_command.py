#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_msgs.msg import ThrusterSpeeds


class SquareCommand(Node):

    NODE_NAME = "sim_move_square"
    MOVE_TOPIC = "/offboard/thruster_speeds"
    FORWARDS = (40, 40, 40, 40, 0, 0, 0, 0)
    BACKWARDS = (-40, -40, -40, -40, 0, 0, 0, 0)
    RIGHT = (-40, 40, 40, -40, 0, 0, 0, 0)  # top view
    LEFT = (40, -40, -40, 40, 0, 0, 0, 0)
    RUN_LOOP_RATE = 1  # Hz

    def __init__(self):
        super().__init__(self.NODE_NAME)
        self._pub = self.create_publisher(ThrusterSpeeds, self.MOVE_TOPIC, 10)
        self.dirs = (self.FORWARDS, self.RIGHT, self.BACKWARDS, self.LEFT)
        self.timer = self.create_timer(1/self.RUN_LOOP_RATE, self.run)
        self.start = self.get_clock().now()
        self.ct = 0

    def run(self):
        data = ThrusterSpeeds()
        self.get_logger().info("HI")
        if (self.get_clock().now() - self.start).to_msg().sec > 5:
            self.ct = (self.ct + 1) % 4
            self.start = self.get_clock().now()
        data.speeds = self.dirs[self.ct]
        self._pub.publish(data)


def main(args=None):
    # Initialize direction vectors
    try:
        rclpy.init(args=args)
        sc = SquareCommand()
        rclpy.spin(sc)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        sys.exit(1)
    finally:
        sc.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
