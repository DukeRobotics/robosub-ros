#!/usr/bin/env python3

from custom_msgs.msg import PIDGain, PIDGains
from custom_msgs.srv import SetPIDGains
import rospy
import copy


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
            PIDGain.AXIS_X: copy.deepcopy(gains),
            PIDGain.AXIS_Y: copy.deepcopy(gains),
            PIDGain.AXIS_Z: copy.deepcopy(gains),
            PIDGain.AXIS_ROLL: copy.deepcopy(gains),
            PIDGain.AXIS_PITCH: copy.deepcopy(gains),
            PIDGain.AXIS_YAW: copy.deepcopy(gains),
        }
        self.pid = {
            PIDGain.LOOP_POSITION: copy.deepcopy(axes),
            PIDGain.LOOP_VELOCITY: copy.deepcopy(axes)
        }

        self.pid_pub = rospy.Publisher('/controls/pid_gains', PIDGains, queue_size=1)
        self.set_pid = rospy.Service('/controls/set_pid_gains', SetPIDGains, self.handle_set_pid)

        rospy.loginfo("pid_sim initialized")
        self.pid_publish_loop()

    def pid_publish_loop(self):
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            pid_gains = PIDGains()

            for loop, axes in self.pid.items():
                for axis, gains in axes.items():
                    for gain, value in gains.items():
                        pid_gain = PIDGain()
                        pid_gain.loop = loop
                        pid_gain.axis = axis
                        pid_gain.gain = gain
                        pid_gain.value = value

                        pid_gains.pid_gains.append(pid_gain)

            self.pid_pub.publish(pid_gains)

            rate.sleep()

    def handle_set_pid(self, req):
        for pid_gain in req.pid_gains:
            self.pid[pid_gain.loop][pid_gain.axis][pid_gain.gain] = pid_gain.value
        return {'success': True, 'message': 'Dummy message'}


def main():
    try:
        PIDSim()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
