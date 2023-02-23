#!/usr/bin/env python3

import rospy
from custom_msgs.srv import StartLaunch, StopLaunch
import subprocess
import multiprocessing

# TODO: Define a message type to publish when a node is launched and terminated and import it here


class RemoteLaunchNode:

    def __init__(self):
        rospy.init_node('remote_launch')

        self.processes = {}
        self.terminated_processes = {}

        # TODO: Initialize a publisher to publish when node is launched and terminated using the message type defined above

        rospy.Service('start_node', StartLaunch, self.start_launch)
        rospy.Service('stop_node', StopLaunch, self.stop_launch)
        rospy.loginfo('Remote Launch ready')
        rospy.spin()

    def start_launch(self, req):
        with multiprocessing.Manager() as manager:
            shared_dict = manager.dict()
            process = multiprocessing.Process(target=self.launch_with_callback, args=(req, shared_dict))
            process.start()

            while 'pid' not in shared_dict:
                continue

            # TODO: Publish node launched, including the pid, package, file, and arguments

            return {'pid': shared_dict['pid']}

    def launch_with_callback(self, req, shared_dict):
        exe = 'roslaunch' if req.is_launch_file else 'rosrun'
        rospy.loginfo(f'Executing {exe} {req.package} {req.file} {req.args}')
        if req.args == ['']:  # No arguments provided
            proc = subprocess.Popen([exe, req.package, req.file])
        else:
            proc = subprocess.Popen([exe, req.package, req.file] + req.args)
        shared_dict['pid'] = proc.pid
        self.processes[int(proc.pid)] = proc
        proc.wait()
        self.terminated_processes.append(proc.pid)
        rospy.loginfo(f'Terminating {int(proc.pid)} {req.package} {req.file} {req.args}')
        # TODO: Publish node terminated, including the pid, package, file, and arguments
        return

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
