#!/usr/bin/env python3

import rospy
from custom_msgs.srv import StartLaunch, StopLaunch
from custom_msgs.msg import RemoteLaunchInfo
import subprocess
import multiprocessing

# TODO: Define a message type to publish when a node is launched and terminated and import it here


class RemoteLaunchNode:

    def __init__(self):
        rospy.init_node('remote_launch')

        self.processes = {}
        self.terminated_processes = []

        self.publisher = rospy.Publisher('remote_launch', RemoteLaunchInfo, queue_size=10)

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
            rli_msg = RemoteLaunchInfo()

            exe = 'roslaunch' if req.is_launch_file else 'rosrun'
            rli_msg.msg_type = 'Executing ' + exe

            rli_msg.pid = shared_dict['pid']
            rli_msg.package = req.package
            rli_msg.file = req.file
            rli_msg.args = req.args

            self.publisher.publish(rli_msg)

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
        rli_msg = RemoteLaunchInfo()

        rli_msg.msg_type = "Terminating"
        rli_msg.pid = proc.pid
        rli_msg.package = req.package
        rli_msg.file = req.file
        rli_msg.args = req.args

        self.publisher.publish(rli_msg)

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
