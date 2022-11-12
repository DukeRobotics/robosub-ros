#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from custom_msgs.action import Saleae
from custom_msgs.msg import HydrophoneSet
import pandas as pd
import saleae
import resource_retriever as rr
from datetime import datetime


class SaleaeInterface(Node):

    NODE_NAME = "saleae"
    ACTION_NAME = "call_saleae"
    SALEAE_SETTINGS = '/root/docker-build/Logic/Settings/settings.xml'
    CAPTURE_DURATION = 2
    HYDROPHONE_SET = {HydrophoneSet.GUESS: [0, 1, 2, 3], HydrophoneSet.PROCESS: [4, 5, 6, 7]}
    FILE_EXTENSIONS = {HydrophoneSet.GUESS: "_guess", HydrophoneSet.PROCESS: "_processing"}

    def __init__(self):
        super().__init__(self.NODE_NAME)
        self.server = ActionServer(self, Saleae, self.ACTION_NAME, self.execute)

        # Delete the saleae settings file to get around this issue:
        # https://github.com/saleae/SaleaeSocketApi/issues/14
        # if os.path.exists(self.SALEAE_SETTINGS):
        #   os.remove(self.SALEAE_SETTINGS)

        self.saleae = saleae.Saleae(args='-disablepopups -socket')
        rclpy.get_logger().info("Saleae started")

    def publish_feedback(self, goal, stage, total_stages, msg):
        feedback = Saleae.Feedback()
        feedback.curr_stage = stage
        feedback.total_stages = total_stages
        feedback.message = msg
        goal.publish_feedback(feedback)

    def format_csv(self, file):
        df = pd.read_csv(file)
        df.drop([0])
        df.drop([0], axis=1)
        df.to_csv(file, index=False)

    def execute(self, goal):
        self.saleae.set_capture_seconds(goal.capture_duration)

        time_now = datetime.now()
        export_name = "date_" + time_now.strftime("%m_%d_%Y_%H_%M_%S") + self.FILE_EXTENSIONS[goal.hydrophone_set.type]
        package_path = 'package://acoustics/data/' + export_name + '_({0}).csv'
        save_paths = []

        for i in range(goal.capture_count):
            self.publish_feedback(goal, i + 1, goal.capture_count + 1, f"Starting capture {i}")
            save_paths.append(package_path.format(i))
            save_path = rr.get_filename(package_path.format(i), use_protocol=False)
            rospy.loginfo("Path: " + save_path)
            self.saleae.export_data2(save_path, analog_channels=self.HYDROPHONE_SET[goal.hydrophone_set.type])
            self.format_csv(save_path)

        self.publish_feedback(goal, goal.capture_count + 1, goal.capture_count + 1, "Saleae capture complete")

        goal.succeed()
        return Saleae.Result(file_paths=save_paths)


def main(args=None):
    try:
        rclpy.init(args=args)
        saleae = SaleaeInterface()
        rclpy.spin(saleae)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        raise
    finally:
        saleae.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
