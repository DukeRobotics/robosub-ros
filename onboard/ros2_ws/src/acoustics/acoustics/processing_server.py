#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from custom_msgs.action import AcousticsProcessing
from acoustics.cross_corr_fft import AcousticProcessor


class ProcessingServer(Node):

    NODE_NAME = "acoustics_processor"
    ACTION_NAME = "process_acoustics"

    def __init__(self):
        super().__init__(self.NODE_NAME)
        self.server = ActionServer(self, AcousticsProcessing, self.ACTION_NAME, self.execute)
        self.total_count = 0

    def publish_feedback(self, goal, success_count, process_count):
        feedback = AcousticsProcessing.Feedback()
        feedback.process_count = process_count
        feedback.success_count = success_count
        feedback.total_count = self.total_count
        goal.publish_feedback(feedback)

    def get_result(self, final_ccwh, valid_count, success_count):
        result = AcousticsProcessing.Result()
        result.hz_angle = final_ccwh
        result.valid_count = valid_count
        result.success_count = success_count
        result.total_count = self.total_count
        return result

    def execute(self, goal):
        self.total_count = len(goal.file_paths) * (2 if goal.if_double else 1)
        guess = (goal.guess.x, goal.guess.y, goal.guess.z)
        processor = AcousticProcessor(goal, goal.file_paths, goal.if_double,
                                      goal.samp_f, goal.tar_f, guess, self.publish_feedback)
        final_ccwh, valid_count, success_count = processor.run()
        goal.succeed()
        return self.get_result(final_ccwh, valid_count, success_count)


def main(args=None):
    try:
        rclpy.init(args=args)
        server = ProcessingServer()
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
