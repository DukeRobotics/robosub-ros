from task import Task
import rospy


class LogTask(Task):

    def __init__(self, level, message):
        super(LogTask, self).__init__()
        self.level = level
        self.message = message

    def _on_task_run(self):
        if self.level == "DEBUG":
            rospy.logdebug(self.message)

        elif self.level == "INFO":
            rospy.loginfo(self.message)

        elif self.level == "WARN":
            rospy.logwarn(self.message)

        elif self.level == "ERROR":
            rospy.logerr(self.message)

        elif self.level == "FATAL":
            rospy.logfatal(self.message)

        else:
            rospy.logwarn("Incorrect logging level specified")
