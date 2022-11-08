#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from custom_msgs.action import AcousticsGuess
from acoustics.cheap_cross_cor import Guess


class GuessServer(Node):

    NODE_NAME = "acoustics_guesser"
    ACTION_NAME = "guess_acoustics"

    def __init__(self):
        super().__init__(self.NODE_NAME)
        self.server = ActionServer(self, AcousticsGuess, self.ACTION_NAME, self.execute)

    def publish_feedback(self, goal, curr_stage, total_stages, msg):
        feedback = AcousticsGuess.Feedback()
        feedback.curr_stage = curr_stage
        feedback.total_stages = total_stages
        feedback.message = msg
        goal.publish_feedback(feedback)

    def get_result(self, point):
        result = AcousticsGuess.Result()
        result.guess.x = point[0]
        result.guess.y = point[1]
        result.guess.z = point[2]
        return result

    def execute(self, goal):
        point = Guess(goal, goal.file_paths, goal.samp_f, goal.tar_f, self.publish_feedback).run()
        goal.succeed()
        return self.get_result(point)


def main(args=None):
    try:
        rclpy.init(args=args)
        server = GuessServer()
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
