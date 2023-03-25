#!/usr/bin/env python3

import rospy
from custom_msgs.srv import StartLaunch, StopLaunch
from custom_msgs.msg import RemoteLaunchInfo
import subprocess
from enum import Enum


class RemoteLaunchMessageType(Enum):
    EXECUTING = 0
    TERMINATING = 1


class RemoteLaunchNode:

    def __init__(self):
        rospy.init_node('remote_launch')

        self.processes = {}
        self.terminated_processes = []

        self.publisher = rospy.Publisher('remote_launch', RemoteLaunchInfo, queue_size=10)

        rospy.Service('start_node', StartLaunch, self.start_launch)
        rospy.Service('stop_node', StopLaunch, self.stop_launch)
        rospy.loginfo('Remote Launch ready')
        self.check_for_terminated_processes()

    def check_for_terminated_processes(self):
        while not rospy.is_shutdown():
            for pid in self.processes:
                if self.processes[pid]["process"].poll() is not None and pid not in self.terminated_processes:
                    # Process has terminated, so add it to the list of terminated processes
                    # and publish termination messsage
                    self.terminated_processes.append(pid)
                    self.publish_message(pid, RemoteLaunchMessageType.TERMINATING)

    def start_launch(self, req):
        exe = 'roslaunch' if req.is_launch_file else 'rosrun'
        rospy.loginfo(f'Executing {exe} {req.package} {req.file} {req.args}')
        if req.args == ['']:  # No arguments provided
            proc = subprocess.Popen([exe, req.package, req.file])
        else:
            proc = subprocess.Popen([exe, req.package, req.file] + req.args)

        # Add message to  process dictionary
        process_dict = {}
        process_dict["package"] = req.package
        process_dict["file"] = req.file
        process_dict["args"] = req.args
        process_dict["is_launch_file"] = req.is_launch_file
        process_dict["process"] = proc

        self.processes[int(proc.pid)] = process_dict

        # Publish executing message
        self.publish_message(proc.pid, RemoteLaunchMessageType.EXECUTING)

        return {'pid': proc.pid}

    def publish_message(self, pid, type):
        rli_msg = RemoteLaunchInfo()

        exe = "roslaunch" if self.processes[pid]["is_launch_file"] else "rosrun"
        type_str = "Executing " + exe if type == RemoteLaunchMessageType.EXECUTING else "Terminating"

        rli_msg.msg_type = type_str

        rli_msg.pid = pid
        rli_msg.package = self.processes[pid]["package"]
        rli_msg.file = self.processes[pid]["file"]
        rli_msg.args = self.processes[pid]["args"]

        self.publisher.publish(rli_msg)

    def stop_launch(self, req):
        if req.pid in self.terminated_processes:
            return {'success': True}

        if req.pid not in self.processes:
            return {'success': False}

        if self.processes[req.pid].poll() is None:
            self.processes[req.pid].terminate()
            self.processes[req.pid].wait()
            self.processes.pop(req.pid)
            return {'success': True}

        return {'success': False}


if __name__ == '__main__':
    RemoteLaunchNode()
