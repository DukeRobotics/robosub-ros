#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from custom_msgs.action import AcousticsGuess, \
    AcousticsProcessing, \
    Saleae, \
    AcousticsData, \
    AcousticsWrapper
from custom_msgs.msg import HydrophoneSet


class Wrapper(Node):

    NODE_NAME = "acoustics_wrapper"
    ACTION_NAME = "acoustics_wrapper"
    SIM_PARAM = "sim"

    SAMPLING_FREQ = {HydrophoneSet.GUESS: 625000, HydrophoneSet.PROCESS: 625000}
    CAPTURE_COUNT = 4
    CAPTURE_DURATION = 2

    def __init__(self):
        super().__init__(self.NODE_NAME)
        sim = self.declare_parameter(self.SIM_PARAM, False).value
        self.get_logger().info(f"Starting acoustics with sim={sim}")
        if sim:
            self.client_sampling = ActionClient(self, AcousticsData, 'generate_data')
            self.server = ActionServer(self,
                                       AcousticsWrapper,
                                       self.ACTION_NAME,
                                       lambda goal: self.execute(goal, self.generate_data))
        else:
            self.client_sampling = ActionClient(self, Saleae, 'call_saleae')
            self.server = ActionServer(self,
                                       AcousticsWrapper,
                                       self.ACTION_NAME,
                                       lambda goal: self.execute(goal, self.saleae_sampling))

        self.client_guess = ActionClient(self, AcousticsGuess, 'guess_acoustics')
        self.client_processing = ActionClient(self, AcousticsProcessing, 'process_acoustics')

        self.client_sampling.wait_for_server()
        self.client_guess.wait_for_server()
        self.client_processing.wait_for_server()

    def publish_feedback(self, goal_handle, stage, msg):
        feedback = AcousticsWrapper.Feedback()
        feedback.curr_stage = stage
        feedback.total_stages = 4
        feedback.message = msg
        goal_handle.publish_feedback(feedback)

    def saleae_sampling(self, wrapper_goal, hydrophone_set):
        goal = Saleae.Goal()
        goal.hydrophone_set.type = hydrophone_set
        goal.capture_count = self.CAPTURE_COUNT
        goal.capture_duration = self.CAPTURE_DURATION
        return self.client_sampling.send_goal(goal).file_paths

    def generate_data(self, wrapper_goal, hydrophone_set):
        goal = AcousticsData.Goal()
        goal.hydrophone_set.type = hydrophone_set
        goal.samp_f = self.SAMPLING_FREQ[hydrophone_set]
        goal.tar_f = wrapper_goal.tar_f
        goal.location = wrapper_goal.location
        return self.client_sampling.send_goal(goal).file_paths

    def get_guess(self, wrapper_goal, guess_files):
        goal = AcousticsGuess.Goal()
        goal.file_paths = guess_files
        goal.samp_f = self.SAMPLING_FREQ[HydrophoneSet.GUESS]
        goal.tar_f = wrapper_goal.tar_f
        return self.client_guess.send_goal(goal).guess

    def get_processing_angle(self, wrapper_goal, processing_files, guess):
        goal = AcousticsProcessing.Goal()
        goal.file_paths = processing_files
        goal.samp_f = self.SAMPLING_FREQ[HydrophoneSet.PROCESS]
        goal.tar_f = wrapper_goal.tar_f
        goal.guess = guess
        return self.client_processing.send_goal(goal).hz_angle

    def execute(self, goal, sampling_fn):
        self.publish_feedback(goal, 0, "Starting sampling for guess")
        guess_filepaths = sampling_fn(goal, HydrophoneSet.GUESS)
        self.publish_feedback(goal, 1, "Generating data complete, starting guess")

        guess = self.get_guess(goal, guess_filepaths)
        self.publish_feedback(goal, 2, "Guess complete, generating data for processing")

        processing_filepaths = sampling_fn(goal, HydrophoneSet.PROCESS)
        self.publish_feedback(
            goal, 3, "Generating data for processing complete, starting processing")

        processing_result = self.get_processing_angle(goal, processing_filepaths, guess)
        self.publish_feedback(goal, 4, "Processing complete")
        goal.succeed()
        result = AcousticsWrapper.Result()
        result.hz_angle = processing_result

        return result


def main(args=None):
    try:
        rclpy.init(args=args)
        wrapper = Wrapper()
        rclpy.spin(wrapper)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        sys.exit(1)
    finally:
        wrapper.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
