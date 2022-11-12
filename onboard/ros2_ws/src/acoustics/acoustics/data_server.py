#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from custom_msgs.action import AcousticsData
from acoustics.data_sim import DataGenerator


class DataServer(Node):
    NODE_NAME = "acoustics_data_generator"
    ACTION_NAME = "generate_data"

    def __init__(self):
        super().__init__(self.NODE_NAME)
        self.server = ActionServer(self, AcousticsData, self.ACTION_NAME, self.execute)

    def publish_feedback(self, goal, curr_stage, total_stages, msg):  # files saved
        feedback = AcousticsData.Feedback()
        feedback.curr_stage = curr_stage
        feedback.total_stages = total_stages
        feedback.message = msg
        goal.publish_feedback(feedback)

    def execute(self, goal):
        pinger_loc = (goal.location.x, goal.location.y, goal.location.z)
        file_paths = DataGenerator(
            goal,
            goal.samp_f,
            goal.tar_f,
            goal.hydrophone_set.type,
            pinger_loc,
            self.publish_feedback).run()
        self.server.succeed()
        result = AcousticsData.Result()
        result.file_paths = file_paths
        return result


def main(args=None):
    try:
        rclpy.init(args=args)
        server = DataServer()
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        raise
    finally:
        server.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
