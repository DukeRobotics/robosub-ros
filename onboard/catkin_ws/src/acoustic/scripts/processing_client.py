#! /usr/bin/env python

import rospy
from __future__
import print_function
import actionlib
# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import acoustic.msg

def processing_client():
    client = actionlib.SimpleActionClient('processing', acoustic.msg.processingAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = acoustic.msg.processingGoal(filename= , if_double=, version=, samp_f=, tar_f=, guess_x=, guess_y=, guess_z=)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        rospy.init_node('acoustic_processing_client')
        result = processing_client()
        print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
