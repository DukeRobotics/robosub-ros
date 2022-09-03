from smach import Concurrence
import rospy
import smach


class Log(smach.State):
    def _init_(self, text, outcomes=['outcome2']):
        #
        self.text = text

    def execute(self, userdata):
        # ff
        rospy.loginfo(self.text)
        return 'outcome2'


cc = Concurrence(outcomes=['outcomey, succeeded'],
                 default_outcome='outcomey',
                 input_keys=['sm_input'],
                 output_keys=['sm_output'],
                 outcome_map={'succeeded': {'Log1': 'outcome2'}})

with cc:
    Concurrence.add('Log1', Log('log1'))
    Concurrence.add('Log2', Log('log2'))


def main():
    rospy.init_node('smach_example_state_machine')
    sm_top = smach.StateMachine(outcomes=['outcome6'])

    with sm_top:
        smach.StateMachine.add('CON', cc, transitions={'succeeded': 'CON', 'outcomey': 'outcome6'})

    # smach.StateMachine.add('CON', sm_top,
    #                        transitions = {'succeeded':'CON'})

    outcome = sm_top.execute()
