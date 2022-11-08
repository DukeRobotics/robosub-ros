#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_msgs.srv import StartLaunch, StopLaunch
import subprocess
import signal


class RemoteLaunch(Node):

    NODE_NAME = 'remote_launch'

    def __init__(self):
        super().__init__(self.NODE_NAME)
        self.processes = {}
        self.create_service(StartLaunch, 'start_node', self.start_launch)
        self.create_service(StopLaunch, 'stop_node', self.stop_launch)
        self.get_logger().info('Remote Launch ready')

    def start_launch(self, req, res):
        cmd = 'ros2'
        exe = 'launch' if req.is_launch_file else 'run'
        self.get_logger().info(f'Executing {cmd} {exe} {req.package} {req.file} {req.args}')
        if not req.args:  # No arguments provided
            proc = subprocess.Popen([cmd, exe, req.package, req.file])
        else:
            proc = subprocess.Popen([cmd, exe, req.package, req.file] + req.args)
        self.processes[int(proc.pid)] = proc
        res.pid = int(proc.pid)
        return res

    def stop_launch(self, req, res):
        res.success = False
        if req.pid not in self.processes:
            return res
        if self.processes[req.pid].poll() is None:
            try:
                self.processes[req.pid].send_signal(signal.SIGINT)
                self.processes[req.pid].wait(timeout=30)
            except subprocess.TimeoutExpired:
                # TODO: Cannot stop processes that were ran with ros2 run
                self.get_logger().error(f'Termination of {req.pid} timed out.')
                return res
            self.processes.pop(req.pid)
            res.success = True
        return res


def main(args=None):
    try:
        rclpy.init(args=args)
        remote_launch = RemoteLaunch()
        rclpy.spin(remote_launch)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        raise
    finally:
        remote_launch.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
