#!/usr/bin/env python3

import subprocess
import os
import rosgraph


class BagRecord():

    def __init__(self, topics, bag_file_path):
        """
        topics: List of strings, where each string is a topic to record
        bag_file_path: String with absolute path to bag file to record, including name of bag file. Must end with .bag
        extension.
        """

        if len(topics) == 0 or ''.join(topics) == "":
            raise ValueError("The list of topics must not be empty.")

        master = rosgraph.Master('rqt_bag_recorder')
        active_topics = [topic for topic, _ in master.getPublishedTopics('')]
        unactive_provided_topics = [topic for topic in topics if topic not in active_topics]
        if len(unactive_provided_topics) > 0:
            raise RuntimeError(('The following topics are not active so cannot be recorded in a bagfile: ' +
                                str(unactive_provided_topics)))

        self.topics = topics

        if (bag_file_path != "" and os.path.splitext(bag_file_path)[1] == ".bag"
                and os.path.exists(os.path.dirname(bag_file_path))):
            self.bag_file_path = bag_file_path
        else:
            raise ValueError("The bag file path must not be empty, end in the the '.bag' file extension, and the \
                                directory of the path must exist.")

        self.proc = None

    def record(self, optional_args=""):
        """
        Starts recording the bag file.
        optional_args: String with arguments to add to rosbag record command. Not required.
        """

        bag_command = ['rosbag', 'record']
        bag_command.extend(self.topics)
        bag_command.extend(["-O", self.bag_file_path])

        if optional_args:
            bag_command.extend(optional_args.split(" "))

        self.proc = subprocess.Popen(bag_command)
        return {'pid': int(self.proc.pid)}

    def stop(self):
        """
        Stops recording the bag file.
        """

        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            self.proc.wait()
            return {'success': True}

        return {'success': False}

    def __del__(self):
        self.stop()
