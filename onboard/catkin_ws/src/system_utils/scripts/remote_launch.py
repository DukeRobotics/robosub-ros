#!/usr/bin/env python3

import rospy
from custom_msgs.srv import StartLaunch, StopLaunch
import subprocess
import multiprocessing


class RemoteLaunchNode:

    def __init__(self):
        rospy.init_node('remote_launch')

        self.processes = {}

        rospy.Service('start_node', StartLaunch, self.start_launch)
        rospy.Service('stop_node', StopLaunch, self.stop_launch)
        rospy.loginfo('Remote Launch ready')
        rospy.spin()

    def start_launch(self, req):
        process = multiprocessing.Process(target=self.launch_with_callback, args=(req,))
        process.start()
        process.join()

    def launch_with_callback(self, req):
        exe = 'roslaunch' if req.is_launch_file else 'rosrun'
        rospy.loginfo(f'Executing {exe} {req.package} {req.file} {req.args}')
        if req.args == ['']:  # No arguments provided
            proc = subprocess.Popen([exe, req.package, req.file])
        else:
            proc = subprocess.Popen([exe, req.package, req.file] + req.args)
        self.processes[int(proc.pid)] = proc
        proc.wait()
        rospy.loginfo(f'Terminating {int(proc.pid)} {req.package} {req.file} {req.args}')
        return
        # return {'pid': int(proc.pid)}

    def stop_launch(self, req):
        if req.pid not in self.processes:
            return {'success': False}

        if self.processes[req.pid].poll() is None:
            self.processes[req.pid].terminate()
            self.processes[req.pid].wait()
            self.processes.pop(req.pid)
            return {'success': True}
        return {'success': False}

    # def log_status(self, req):
    #     rospy.loginfo(f'Terminating {req.pid} {req.package} {req.file} {req.args}')


if __name__ == '__main__':
    RemoteLaunchNode()
