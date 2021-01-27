#!/usr/bin/env python

import rospy
import actionlib
from custom_msgs.msg import AcousticsGuessGoal, AcousticsGuessAction, \
    AcousticsProcessingGoal, AcousticsProcessingAction, \
    SaeleaeAction, SaeleaeFeedback, SaeleaeResult, SaeleaeGoal
import saleae
import sys
import os



class AcousticsWrapper:

    NODE_NAME = "saeleae"
    ACTION_NAME = "call_saeleae"
    IP_ADDRESS = "localhost"
    PORT = 10429
    CAPTURE_COUNT = 1
    CAPTURE_DURATION = .004

    def __init__(self):
        rospy.init_node(self.NODE_NAME)
        self.server = actionlib.SimpleActionServer(self.ACTION_NAME, SaeleaeAction, self.execute, False)
        self.server.start()
        rospy.spin()

    def publish_feedback(self, stage):
        feedback = SaeleaeFeedback()
        feedback.curr_stage = stage
        feedback.total_stage = 2
        self.server.publish_feedback(feedback)

    def publish_result(self, return_file):
        result = SaeleaeResult()
        SaeleaeResult.return_file_name = return_file
        self.server.set_succeeded(result)

    def validate_path(path, argument_name):
        if path is not None:
            if not os.path.isdir(path):
                print('the specified ' + argument_name + ' directory does not exist or is invalid')
                print('you specified: ' + path)
                quit()

    def execute(self, goal):
        s = saleae.Saleae(self.IP_ADDRESS, self.PORT)
        analyzer_names = []
        save_file = SaeleaeGoal.save_path
        self.publish_feedback(1)
        if SaeleaeGoal.hydrophone_set == 1:
            s.export_data2(SaeleaeGoal.save_path, digital_channels=HYDROPHONE_SET_1, analog_channels=None,
                           time_span=[0, 0.04], format='csv', column_headers=False, delimiter='comma',
                           timestamp='time_stamp', display_base='separate', rows_per_change=True)
        if SaeleaeGoal.hydrophone_set == 2:
            s.export_data2(SaeleaeGoal.save_path, digital_channels=HYDROPHONE_SET_2, analog_channels=None,
                           time_span=[0, 0.04], format='csv', column_headers=False, delimiter='comma',
                           timestamp='time_stamp', display_base='separate', rows_per_change=True)
        self.publish_feedback(2)
        self.publish_result(save_file)
"""
        for x in range(self.CAPTURE_COUNT):
            # set capture duration
            s.set_capture_seconds(self.CAPTURE_DURATION)
            # start capture. Only save to disk if the --save-captures option was specified.
            if SaeleaeGoal.capture_directory != '':
                file_name = '{0}.logicdata'.format(x)
                save_path = os.path.join(SaeleaeGoal.capture_directory, file_name)
                print('starting capture and saving to ' + save_path)
                s.capture_to_file(save_path)
            else:
                # currently, the python library doesn't provide a CAPTURE command that blocks until an ACK is received
                s._cmd('CAPTURE')
            publish_feedback(2)
            # raw export
          #  if SaeleaeGoal.export_file != None:
           #     file_name = '{0}.csv'.format(x)
            #    save_path = os.path.join(sys.path[0], '../data', file_name)
             #   print('exporting data to ' + save_path) #possible feedback location?
              #  s.export_data2(save_path)

            # analyzer export
            if SaeleaeGoal.save_path != None:
                analyzers = s.get_analyzers()
                if analyzers.count == 0:
                    print('Warning: analyzer export path was specified, but no analyzers are present in the capture')
                if SaeleaeGoal.hydrophone_set == 1:
                    for a in range(8):
                        analyzer = analyzers[a]
                        print('exporting analyzer ' + analyzer[0] + ' to ' + save_path)
                        s.export_analyzer(analyzer[1], save_path)
                        analyzer_names.append((analyzer[0], x))
                    publish_feedback(3)
                if SaeleaeGoal.hydrophone_set == 2:
                    for a in range(8,16):
                        analyzer = analyzers[a]
                        print('exporting analyzer ' + analyzer[0] + ' to ' + save_path)
                        s.export_analyzer(analyzer[1], save_path)
                        analyzer_names.append((analyzer[0], x))
                    publish_feedback(3)

               # for analyzer in analyzers:
                #    file_name = '{0}_{1}.csv'.format(x, analyzer[0])
                 #   save_path = os.path.join(SaeleaeGoal.export_analyzers, file_name)
                  #  print('exporting analyzer ' + analyzer[0] + ' to ' + save_path)
                   # s.export_analyzer(analyzer[1], save_path)
                    #analyzer_names.append((analyzer[0],x))
                    """



if __name__ == '__main__':
    AcousticsWrapper()