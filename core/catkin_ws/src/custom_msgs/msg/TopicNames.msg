# This message contains the names of all of the topics we use

std_msgs/Header header

# Sensors
# If changed, also change sensor_fusion\params\main.yaml and data_pub\README.md
string sensors_imu_imu=sensors/imu/imu
# If changed, also change data_pub\README.md
string sensors_imu_mag=sensors/imu/mag
# If changed, also change data_pub\README.md
string sensors_dvl_raw=sensors/dvl/raw
# If changed, also change sensor_fusion\params\main.yaml and data_pub\README.md
string sensors_dvl_odom=sensors/dvl/odom
string sensors_depth=sensors/depth

# Controls
# If changed, also change README.md and controls\README.md
string state=/state
string state_local=/state_local
# If changed, also change controls\README.md
string controls_desired_pose=controls/desired_pose
# If changed, also change controls\README.md
string controls_desired_twist=controls/desired_twist
# If changed, also change controls\README.md and joystick\README.md
string controls_desired_power=controls/desired_power

# The following are all meant to be used with .format(axis)
# If they are changed, also change their variants in controls\launch\position_pid.launch
string controls_enable_pos=/controls/enable/{}_pos
string controls_vel_setpoint=/controls/{}_vel/setpoint
string controls_state_pose=/controls/state/pose/

# The following are all meant to be used with .format(axis)
# If they are changed, also change their variants in controls\launch\velocity_pid.launch
string controls_enable_vel=/controls/enable/{}_vel
string control_effort=/control_effort/{}
string controls_state_twist=/controls/state/twist/{}

# The following are all meant to be used with .format(axis)
string controls_pos_setpoint=/controls/{}_pos/setpoint
string controls_power=/controls/power/{}

# Offboard
# If changed, also change controls\README.md and offboard_comms\README.md
string offboard_thruster_speeds=/offboard/thruster_speeds
# If changed, also change offboard_comms\README.md
string offboard_servo_angles=/offboard/servo_angles
string offboard_pressure=/offboard/pressure

# Joystick
# If changed, also change joystick\launch\joystick_raw.launch
string joystick_raw=joystick/raw

# Simulation
# If changed, also change simulation\README.md
string sim_move=/sim/move
# If changed, also change simulation\README.md
string sim_dvl=/sim/dvl
# If changed, also change simulation\README.md and simulation\personal\scenes\README.md
string sim_pose=/sim/pose
# If changed, also change simulation\README.md
string sim_imu=/sim/imu
string sim_test_array=/sim/test_array
string sim_object_points=/sim/object_points

# Camera
# If changed, also change camera_view\README.md, avt_camera\README.md, and cv\README.md
string camera_image_raw=/camera/{}/image_raw
# If changed, also change avt_camera\README.md
string camera_camera_info=/camera/{}/camera_info

# CV
# If changed, also change cv\README.md and cv\models\models.yaml
string cv_buoy=/cv/buoy
string cv_start_gate_left=/cv/_start_gate/left
string cv_start_tick_left=/cv/_start_tick/left