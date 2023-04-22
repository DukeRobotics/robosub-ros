#!/usr/bin/env python3

import rospy
from custom_msgs.srv import StartLaunch, StopLaunch, GetRunningNodes
from custom_msgs.msg import RemoteLaunchInfo, RunningNode
import subprocess
from threading import Lock


class RemoteLaunchMessageType(Enum):
    EXECUTING = 0
    TERMINATING = 1


class RemoteLaunchNode:

    def __init__(self):
        rospy.init_node('remote_launch')

        self.processes = {}
        self.terminated_processes = []

        self.processes_lock = Lock()

        self.publisher = rospy.Publisher('remote_launch', RemoteLaunchInfo, queue_size=10)

        rospy.Service('start_node', StartLaunch, self.start_launch)
        rospy.Service('stop_node', StopLaunch, self.stop_launch)
        rospy.Service('running_nodes', GetRunningNodes, self.get_running_processes)
        rospy.loginfo('Remote Launch ready')
        self.check_for_terminated_processes()

    def check_for_terminated_processes(self):
        while not rospy.is_shutdown():
            self.processes_lock.acquire()
            for pid in self.processes:
                if self.processes[pid]["process"].poll() is not None and pid not in self.terminated_processes:
                    # Process has terminated, so add it to the list of terminated processes
                    # and publish termination messsage
                    self.terminated_processes.append(pid)
                    self.publish_message(pid, RemoteLaunchInfo.TERMINATING)
            self.processes_lock.release()

    def start_launch(self, req):
        exe = 'roslaunch' if req.is_launch_file else 'rosrun'
        if req.args == ['']:  # No arguments provided
            proc = subprocess.Popen([exe, req.package, req.file])
        else:
            proc = subprocess.Popen([exe, req.package, req.file] + req.args)

        # Add message to process dictionary
        process_dict = {}
        process_dict["package"] = req.package
        process_dict["file"] = req.file
        process_dict["args"] = req.args
        process_dict["is_launch_file"] = req.is_launch_file
        process_dict["process"] = proc

        self.processes_lock.acquire()
        self.processes[int(proc.pid)] = process_dict
        self.processes_lock.release()

        # Publish executing message
        self.publish_message(proc.pid, RemoteLaunchInfo.EXECUTING)

        return {'pid': proc.pid}

    def publish_message(self, pid, type):
        rli_msg = RemoteLaunchInfo()

        exe = "roslaunch" if self.processes[pid]["is_launch_file"] else "rosrun"
        type_str = "Executing " + exe if type == RemoteLaunchInfo.EXECUTING else "Terminating"

        rli_msg.msg_type = type

        running_node_msg = RunningNode()

        running_node_msg.pid = pid
        running_node_msg.package = self.processes[pid]["package"]
        running_node_msg.file = self.processes[pid]["file"]
        running_node_msg.args = self.processes[pid]["args"]
        running_node_msg.file_type = \
            RunningNode.ROSLAUNCH if self.processes[pid]["is_launch_file"] else RunningNode.ROSRUN

        rli_msg.running_node_info = running_node_msg

        self.publisher.publish(rli_msg)
        rospy.loginfo(f'{type_str} {self.processes[pid]["package"]} '
                      f'{self.processes[pid]["file"]} {self.processes[pid]["args"]}'
                      )

    def stop_launch(self, req):
        if req.pid in self.terminated_processes:
            return {'success': True}

        if req.pid not in self.processes:
            return {'success': False}

        if self.processes[req.pid]["process"].poll() is None:
            self.processes[req.pid]["process"].terminate()
            self.processes[req.pid]["process"].wait()
            return {'success': True}

        return {'success': False}

    def get_running_processes(self, _):
        running_nodes = []

        for pid in self.processes:
            if pid not in self.terminated_processes:
                running_node_msg = RunningNode()

                running_node_msg.pid = pid
                running_node_msg.package = self.processes[pid]["package"]
                running_node_msg.file = self.processes[pid]["file"]
                running_node_msg.args = self.processes[pid]["args"]
                running_node_msg.file_type = \
                    RunningNode.ROSLAUNCH if self.processes[pid]['is_launch_file'] else RunningNode.ROSRUN

                running_nodes.append(running_node_msg)

        return {'running_nodes_msgs': running_nodes}


if __name__ == '__main__':
    RemoteLaunchNode()
