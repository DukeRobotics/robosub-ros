from paramiko.client import SSHClient, WarningPolicy

class GUISSHClient():
    
    def __init__(self, sim_mode=True):
        self.sim_mode = sim_mode
        self.client = SSHClient()
        self.client.load_system_host_keys()
        self.client.set_missing_host_key_policy(WarningPolicy())
        self.linear_x = self.linear_y = self.linear_z = self.angular_x = self.angular_y = self.angular_z = 0
        self.command = ""

    def __execute(self):
        ip = '192.168.1.1'
        self.client.connect(ip, port=2200,username='root',password='robotics', allow_agent=False)
        self.client.exec_command("source /opt/ros/melodic/setup.bash")
        self.client.exec_command(self.command)

    def stop(self):
        self.client.close()
    
    def set_linear_x(self, linear_x):
        self.linear_x = linear_x

    def set_linear_y(self, linear_y):
        self.linear_y = linear_y

    def set_linear_z(self, linear_z):
        self.linear_z = linear_z

    def set_angular_x(self, angular_x):
        self.angular_x = angular_x

    def set_angular_y(self, angular_y):
        self.angular_y = angular_y

    def set_angular_z(self, angular_z):
        self.angular_z = angular_z

    def execute_pose(self):
        self.command = ("rostopic pub -r 15 /controls/desired_pose geometry_msgs/Pose " +
        "'{{linear: {{x: {lin_x}, y: {lin_y}, z: {lin_z}}}, angular: {{x: {ang_x}, y: {ang_y}, z: {ang_z}}} }}'").format(lin_x = self.linear_x, lin_y=self.linear_y, lin_z=self.linear_z, 
        ang_x=self.angular_x, ang_y=self.angular_y, ang_z=self.angular_z)

        self.__execute()

    def execute_twist(self):
        self.command = ("rostopic pub -r 15 /controls/desired_twist geometry_msgs/Twist " +
        "'{{linear: {{x: {lin_x}, y: {lin_y}, z: {lin_z}}}, angular: {{x: {ang_x}, y: {ang_y}, z: {ang_z}}} }}'").format(lin_x = self.linear_x, lin_y=self.linear_y, lin_z=self.linear_z, 
        ang_x=self.angular_x, ang_y=self.angular_y, ang_z=self.angular_z)

        self.__execute()

    def execute_launch_file(self):
        self.command = "roslaunch execute motion.launch"
        if self.sim_mode:
            self.command += " sim:=True"
        
        self.__execute()