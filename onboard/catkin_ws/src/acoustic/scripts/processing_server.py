#! /usr/bin/env python

# import rospy
# import actionlib
# import acoustic.msg
import os
import signal
import pathlib
import subprocess

# class processingAction(object):
# # create messages that are used to publish feedback/result
#     _feedback = acoustic.msg.processingFeedback()
#     _result = acoustic.msg.processingResult()
#
#     def __init__(self, name):
#         self._action_name = name
#         self._as = actionlib.SimpleActionServer(self._action_name, acoustic.msg.processingAction, execute_cb=self.execute_cb, auto_start = False)
#         self._as.start()
#
#     def execute_cb(self, goal):
#         # helper variables
#         r = rospy.Rate(1)
#         success = True
#
#         # initial feedback
#         self._feedback.total_count = int(goal.version*(2 if goal.if_double else 1))
#         self._feedback.process_count = 0
#         self._feedback.success_count = 0
#
#         # publish info to the console for the user
#         rospy.loginfo('%s: Executing, running cross_corr_func with %s, %r, %i, %i, %i, (%f, %f, %f)' % (goal.filename, goal.if_double, goal.version, goal.samp_f, goal.tar_f, goal.guess_x, goal.guess_y, goal.guess_z))
#
#         process = subprocess.Popen(["python3", os.path.join(pathlib.Path(__file__).parent.absolute(), "cross_corr_fft.py")], stdout = subprocess.PIPE, preexec_fn=os.setsid)
#
#         while not process.poll():
#             if self._as.is_preempt_requested():
#                 os.killpg(os.getpgid(process.pid), signal.SIGTERM)
#                 rospy.loginfo('%s: Preempted' % self._action_name)
#                 self._as.set_preempted()
#                 success = False
#                 break
#
#             line = process.stdout.readline().strip()
#             if "final horizontal angle" in line:
#                 self._result.hz_angle = line.split()[-1]
#             if "valid count" in line:
#                 self._result.valid_count = line.split()[-1]
#             if "horizontal angle" in line or "invalid" in line:
#                 if "horizontal angle" in line:
#                     self._feedback.success_count += 1
#                 self._feedback.process_count += 1
#                 self._as.publish_feedback(self._feedback)
#
#         if success:
#             self._result.total_count = self._feedback.total_count
#             self._result.success_count = self._feedback.success_count
#             rospy.loginfo('%s: Succeeded' % self._action_name)
#             self._as.set_succeeded(self._result)


if __name__ == '__main__':
    # rospy.init_node('acoustic_processing')
    # server = processingAction(rospy.get_name())
    # rospy.spin()
    process = subprocess.Popen(["python3", os.path.join(pathlib.Path(__file__).parent.absolute(), "cross_corr_fft.py", "/Users/estellehe/Documents/duke/senior/IndepStudy/wilson_range_dive/625k_40k_3_-3_-4.csv")], stdout = subprocess.PIPE, preexec_fn=os.setsid)

    while not process.poll():
        line = process.stdout.readline().strip()
        if "final horizontal angle" in line:
            print(line.split()[-1])
        if "valid count" in line:
            print(line.split()[-1])
        if "horizontal angle" in line or "invalid" in line:
            if "horizontal angle" in line:
                print("success")
            print("count")
