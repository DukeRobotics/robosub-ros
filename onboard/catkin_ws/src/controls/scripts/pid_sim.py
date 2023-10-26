#!/usr/bin/env python3

from custom_msgs.msg import PIDGain
from custom_msgs.srv import SetPIDGains
import rospy


class PIDSim():

    def __init__(self):
        rospy.init_node('pid_sim')

        gains: dict[int, float] = {
            PIDGain.GAIN_KP: 0,
            PIDGain.GAIN_KI: 0,
            PIDGain.GAIN_KD: 0,
            PIDGain.GAIN_FF: 0
        }
        axes = {
            PIDGain.AXIS_X: gains,
            PIDGain.AXIS_Y: gains,
            PIDGain.AXIS_Z: gains,
            PIDGain.AXIS_ROLL: gains,
            PIDGain.AXIS_PITCH: gains,
            PIDGain.AXIS_YAW: gains,
        }
        self.pid = {
            PIDGain.POSITION_PID: axes,
            PIDGain.VELOCITY_PID: axes
        }

        print(self.pid)

        self.pid_pub = rospy.Publisher('current_pid', PIDGain, queue_size=1)
        self.set_pid = rospy.Service('set_pid', SetPIDGains, self.handle_set_pid)

        rospy.loginfo("pid_sim initialized")
        self.pid_publish_loop()

    def pid_publish_loop(self):
        rate = rospy.Rate(20)
        pid_gain = PIDGain()
        while not rospy.is_shutdown():
            for pid_loop, axes in enumerate(self.pid):
                pid_gain.pid_loop = pid_loop
                for axis, gains in enumerate(axes):
                    print(axes)
                    pid_gain.axis = axis
                    for gain_type, gain in enumerate(gains):
                        pid_gain.gain_type = gain_type
                        pid_gain.gain = gain

                        print(pid_gain)

                        self.pid_pub.publish(pid_gain)
            rate.sleep()

    def handle_set_pid(self, req):
        for pid_gain in req.pid_gains:
            self.pid[pid_gain.pid_loop][pid_gain.axis][pid_gain.gain_type] = pid_gain.gain
        return {'success': True}


def main():
    try:
        PIDSim()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
