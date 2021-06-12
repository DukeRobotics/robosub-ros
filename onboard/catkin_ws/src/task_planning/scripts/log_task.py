from task import Task
import rospy


class LogTask(Task):

    def __init__(self, level, message):
        super(LogTask, self).__init__()
        self.level = level
        self.message = message

        self.logDict = {
            "DEBUG": rospy.logdebug,
            "INFO": rospy.loginfo,
            "WARN": rospy.logwarn,
            "ERROR": rospy.logerr,
            "FATAL": rospy.logfatal
        }

    def _on_task_run(self):
        if self.level in self.logDict:
            self.logDict[self.level](self.message)

        else:
            rospy.logwarn("Incorrect logging level specified")

        self.finish()
