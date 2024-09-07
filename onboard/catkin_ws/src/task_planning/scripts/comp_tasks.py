import rospy
import math
import copy
import numpy as np

from transforms3d.euler import quat2euler

from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
from custom_msgs.msg import ControlTypes

from task import Task, task
import move_tasks
import cv_tasks
from utils import geometry_utils
from task import Yield

from enum import Enum

from interface.state import State
from interface.cv import CV
from interface.controls import Controls

from utils.coroutine_utils import sleep


# TODO: move stablize() to move_tasks.py
#
# TODO: see if we can remove sleep() since we already have sleep() in coroutine_utils.py
#
# TODO: create a common skeleton @task class/interface with all the common functions to remove redundancy:
# - move_x
# - move_y
# - move_z
# - correct_x
# - correct_y
# - correct_z
# - correct_yaw
# - correct_roll_and_pitch
# - get_yaw_correction
# - ...
# These implementations can be overridden by the tasks that uses that interface.
#
# TODO: look into creating common higher level routines:
# - yaw_to_cv_object
# - spiral search (e.g. spiral_bin_search)
# - logarithmic search (stretch)
# - track and align with object center for bottom camera (e.g. search_for_bins & center_path_marker)
# - track and move toward CV object (e.g. move_to_pink_bins & move_to_buoy)
#     - takes in the termination condition function as a parameter
#     - can improve on cv_tasks.move_to_cv_obj implementation (or replace it completely)


RECT_HEIGHT_METERS = 0.3048


@task
async def gate_style_task(self: Task, depth_level=0.9) -> Task[None, None, None]:
    """
    Complete two full barrel rolls.
    """

    rospy.loginfo("Started gate style task")

    DEPTH_LEVEL = State().orig_depth - depth_level

    async def sleep(secs):
        duration = rospy.Duration(secs)
        start_time = rospy.Time.now()
        while start_time + duration > rospy.Time.now():
            await Yield()

    async def roll():
        power = Twist()
        power.angular.x = 1
        Controls().publish_desired_power(power)
        rospy.loginfo("Published roll power")

        await sleep(2.25)

        rospy.loginfo("Completed roll")

        Controls().publish_desired_power(Twist())
        rospy.loginfo("Published zero power")

        await sleep(2)

        rospy.loginfo("Completed zero")

    await move_tasks.depth_correction(DEPTH_LEVEL, parent=self)
    await roll()
    State().reset_pose()
    await roll()
    State().reset_pose()
    await sleep(3)
    await move_tasks.depth_correction(DEPTH_LEVEL, parent=self)
    await sleep(3)

    imu_orientation = State().imu.orientation
    euler_angles = quat2euler([imu_orientation.w, imu_orientation.x, imu_orientation.y, imu_orientation.z])
    roll_correction = -euler_angles[0]
    pitch_correction = -euler_angles[1]

    rospy.loginfo(f"Roll, pitch correction: {roll_correction, pitch_correction}")
    await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, roll_correction, pitch_correction, 0),
                                        parent=self)
    State().reset_pose()
    rospy.loginfo("Reset orientation")


@task
async def buoy_task(self: Task, turn_to_face_buoy=False, depth=0.7) -> Task[None, None, None]:
    """
    Circumnavigate the buoy. Requires robot to have submerged 0.5 meters.
    """

    rospy.loginfo("Starting buoy task")

    DEPTH_LEVEL = State().orig_depth - depth

    async def correct_y():
        await cv_tasks.correct_y("buoy", parent=self)

    async def correct_z():
        await cv_tasks.correct_z(prop="buoy", parent=self)

    async def correct_depth():
        await move_tasks.correct_depth(desired_depth=DEPTH_LEVEL, parent=self)
    self.correct_depth = correct_depth

    async def move_x(step=1):
        await move_tasks.move_x(step=step, parent=self)

    def get_step_size(dist, dist_threshold):
        if dist > 3:
            return 2
        elif dist > 2:
            return 1
        elif dist > 1.5:
            return 0.5
        else:
            return min(dist - dist_threshold + 0.1, 0.25)

    async def move_to_buoy(buoy_dist_threshold=1):
        buoy_dist = CV().cv_data["buoy"].coords.x
        await correct_y()
        await correct_depth()

        while buoy_dist > buoy_dist_threshold:
            await move_x(step=get_step_size(buoy_dist, buoy_dist_threshold))
            rospy.loginfo(f"Buoy dist: {CV().cv_data['buoy'].coords.x}")
            await correct_y()
            if buoy_dist < 3:
                await correct_z()
            else:
                await correct_depth()

            await Yield()
            buoy_dist = CV().cv_data["buoy"].coords.x
            rospy.loginfo(f"Buoy dist: {CV().cv_data['buoy'].coords.x}")

        await correct_z()

    await move_to_buoy()

    start_imu_orientation = copy.deepcopy(State().imu.orientation)
    start_imu_euler_angles = quat2euler(geometry_utils.geometry_quat_to_transforms3d_quat(start_imu_orientation))

    def get_yaw_correction():
        cur_imu_orientation = copy.deepcopy(State().imu.orientation)
        cur_imu_euler_angles = quat2euler(geometry_utils.geometry_quat_to_transforms3d_quat(cur_imu_orientation))

        return start_imu_euler_angles[2] - cur_imu_euler_angles[2]

    async def correct_yaw():
        yaw_correction = get_yaw_correction()
        rospy.loginfo(f"Yaw correction: {yaw_correction}")
        sign = 1 if yaw_correction > 0.1 else (-1 if yaw_correction < -0.1 else 0)
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(0, 0, 0, 0, 0, yaw_correction + (sign * 0.1)),
            keep_level=True,
            parent=self
        )
        rospy.loginfo("Corrected yaw")
    self.correct_yaw = correct_yaw

    async def move_with_directions(directions, correct_yaw=True):
        await move_tasks.move_with_directions(directions, correct_yaw=correct_yaw, correct_depth=True, parent=self)

    if turn_to_face_buoy:
        def get_step_size_move_away(dist, dist_threshold):
            if dist < 0.75:
                return -0.5
            else:
                return max(dist - dist_threshold - 0.1, -0.25)

        async def move_away_from_buoy(buoy_dist_threshold=1.0):
            rospy.loginfo("Moving away from buoy")
            buoy_dist = CV().cv_data["buoy"].coords.x
            await correct_y()
            await correct_z()
            while buoy_dist < buoy_dist_threshold:
                await move_x(step=get_step_size_move_away(buoy_dist, buoy_dist_threshold))

                rospy.loginfo(f"Buoy dist: {CV().cv_data['buoy'].coords.x}")
                await correct_y()
                await correct_z()
                await Yield()
                buoy_dist = CV().cv_data["buoy"].coords.x
                rospy.loginfo(f"Buoy dist: {CV().cv_data['buoy'].coords.x}")

            rospy.loginfo("Moved away from buoy")

        # Circumnavigate buoy
        for _ in range(4):
            DEPTH_LEVEL = State().depth
            directions = [
                (0, 1.5, 0),
                (1, 0, 0),
            ]
            await move_with_directions(directions, correct_yaw=False)
            await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, 0, 0, -math.radians(90)),
                                                parent=self)
            rospy.loginfo("Yaw 90 deg")
            await move_away_from_buoy()

    else:
        directions = [
            (0, 1.25, 0),
            (2.25, 0, 0),
            (0, -2.5, 0),
            (-2.5, 0, 0),
            (0, 1.25, 0),
        ]
        await move_with_directions(directions, correct_yaw=False)

        await move_to_buoy()


@task
async def after_buoy_task(self: Task):

    DEPTH_LEVEL = State().depth
    latency_threshold = 3

    async def correct_depth():
        await move_tasks.correct_depth(desired_depth=DEPTH_LEVEL, parent=self)
    self.correct_depth = correct_depth

    def is_receiving_cv_data():
        return "path_marker" in CV().cv_data and \
            rospy.Time.now().secs - CV().cv_data["path_marker"].header.stamp.secs < latency_threshold

    def stabilize():
        pose_to_hold = copy.deepcopy(State().state.pose.pose)
        Controls().publish_desired_position(pose_to_hold)

    directions = [
        (0, -1.25, 0),
        (2.25, 0, 0),
        (0, 2.5, 0),
        (-2.5, 0, 0),
        (0, -1.25, 0),
    ]
    circumnavigate_task = move_tasks.move_with_directions(
        directions, correct_yaw=False, correct_depth=True, parent=self)

    while not circumnavigate_task.done:
        circumnavigate_task.step()
        if is_receiving_cv_data():
            stabilize()
            await sleep(5)
            break

        await Yield()

    await move_tasks.move_with_directions([(0, 0, 0, 0, 0, -math.radians(90))], parent=self)

    await align_path_marker(direction=-1, parent=self)

    DEPTH_LEVEL = State().orig_depth - 0.7

    directions = [
        (2, 0, 0),
        (2, 0, 0),
        (2, 0, 0),
        (1, 0, 0)
    ]

    await move_tasks.move_with_directions(directions, correct_yaw=False, correct_depth=True, parent=self)

    found_bins = await spiral_bin_search(parent=self)

    if found_bins:
        await bin_task(parent=self)

    await yaw_to_cv_object('bin_pink_front', direction=1, yaw_threshold=math.radians(15),
                           depth_level=1.0, parent=Task.MAIN_ID)

    await octagon_task(direction=1, parent=self)


@task
async def buoy_to_octagon(self: Task, direction: int = 1, move_forward: int = 0):
    DEPTH_LEVEL = State().orig_depth - 0.7

    rospy.loginfo("Started buoy to octagon")

    async def move_with_directions(directions):
        await move_tasks.move_with_directions(directions, correct_yaw=False, correct_depth=True, parent=self)

    async def correct_depth():
        await move_tasks.correct_depth(desired_depth=DEPTH_LEVEL, parent=self)
    self.correct_depth = correct_depth

    # Move towards octagon
    directions = [
        (0, 2 * direction, 0),
        (0, 2 * direction, 0),
        (0, 2 * direction, 0),
        (0, 1 * direction, 0),
        (move_forward, 0, 0)
    ]
    await move_with_directions(directions)


@task
async def buoy_circumnavigation_power(self: Task, depth=0.7) -> Task[None, None, None]:

    DEPTH_LEVEL = State().orig_depth - depth

    def publish_power():
        power = Twist()
        power.linear.y = 0.9
        power.angular.z = -0.1
        Controls().set_axis_control_type(x=ControlTypes.DESIRED_POWER, y=ControlTypes.DESIRED_POWER,
                                         yaw=ControlTypes.DESIRED_POWER)
        Controls().publish_desired_power(power, set_control_types=False)

    def stabilize():
        pose_to_hold = copy.deepcopy(State().state.pose.pose)
        Controls().publish_desired_position(pose_to_hold)

    async def correct_depth():
        await move_tasks.correct_depth(desired_depth=DEPTH_LEVEL, parent=self)

    for _ in range(4):
        publish_power()
        rospy.loginfo("Publish power")
        await sleep(6)
        rospy.loginfo("Sleep 5 (1)")
        stabilize()
        rospy.loginfo("Stabilized")
        await sleep(5)
        rospy.loginfo("Sleep 5 (2)")
        await correct_depth()
        await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 1, 0, 0, 0, 0), parent=self)


@task
async def initial_submerge(self: Task, submerge_dist: float) -> Task[None, None, None]:
    """
    Submerge the robot a given amount.

    Args:
        submerge_dist: The distance to submerge the robot in meters.
    """

    await move_tasks.move_to_pose_local(
        geometry_utils.create_pose(0, 0, submerge_dist, 0, 0, 0),
        keep_level=True,
        parent=self
    )
    rospy.loginfo(f"Submerged {submerge_dist} meters")

    async def correct_roll_and_pitch():
        imu_orientation = State().imu.orientation
        euler_angles = quat2euler([imu_orientation.w, imu_orientation.x, imu_orientation.y, imu_orientation.z])
        roll_correction = -euler_angles[0] * 1.2
        pitch_correction = -euler_angles[1] * 1.2

        rospy.loginfo(f"Roll, pitch correction: {roll_correction, pitch_correction}")
        await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, roll_correction, pitch_correction, 0),
                                            parent=self)

    await correct_roll_and_pitch()


@task
async def coin_flip(self: Task, depth_level=0.7) -> Task[None, None, None]:
    rospy.loginfo("Started coin flip")
    DEPTH_LEVEL = State().orig_depth - depth_level
    MAXIMUM_YAW = math.radians(30)

    def get_step_size(desired_yaw):
        return min(abs(desired_yaw), MAXIMUM_YAW)

    def get_yaw_correction():
        orig_imu_orientation = copy.deepcopy(State().orig_imu.orientation)
        orig_imu_euler_angles = quat2euler(geometry_utils.geometry_quat_to_transforms3d_quat(orig_imu_orientation))

        cur_imu_orientation = copy.deepcopy(State().imu.orientation)
        cur_imu_euler_angles = quat2euler(geometry_utils.geometry_quat_to_transforms3d_quat(cur_imu_orientation))

        correction = orig_imu_euler_angles[2] - cur_imu_euler_angles[2]

        sign_correction = np.sign(correction)
        desired_yaw = sign_correction * get_step_size(correction)
        rospy.loginfo(f'Coinflip: desired_yaw = {desired_yaw}')

        return correction
        # return desired_yaw

    while abs(yaw_correction := get_yaw_correction()) > math.radians(5):
        rospy.loginfo(f"Yaw correction: {yaw_correction}")
        sign = 1 if yaw_correction > 0.1 else (-1 if yaw_correction < -0.1 else 0)
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(0, 0, 0, 0, 0, yaw_correction + (sign * 0.1)),
            keep_level=True,
            parent=self
        )
        rospy.loginfo("Back to original orientation")

    rospy.loginfo(f"Final yaw correction: {get_yaw_correction()}")

    depth_delta = DEPTH_LEVEL - State().depth
    await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, depth_delta, 0, 0, 0), parent=self)
    rospy.loginfo(f"Corrected depth {depth_delta}")

    rospy.loginfo("Completed coin flip")


@task
async def gate_task_dead_reckoning(self: Task) -> Task[None, None, None]:
    rospy.loginfo("Started gate task")
    DEPTH_LEVEL = State().orig_depth - 0.6
    STEPS = 5

    for _ in range(STEPS):
        await move_tasks.move_x(step=1, parent=self)
        await move_tasks.correct_depth(desired_depth=DEPTH_LEVEL, parent=self)

    rospy.loginfo("Moved through gate")


@task
async def gate_task(self: Task, offset: int = 0, direction: int = 1) -> Task[None, None, None]:
    rospy.loginfo("Started gate task")
    DEPTH_LEVEL = State().orig_depth - 0.7

    async def correct_y(factor=1):
        await cv_tasks.correct_y(prop="gate_red_cw", add_factor=0.2 + offset, mult_factor=factor, parent=self)

    async def correct_z():
        await cv_tasks.correct_z(prop="gate_red_cw", parent=self)

    async def correct_depth():
        await move_tasks.correct_depth(desired_depth=DEPTH_LEVEL, parent=self)
    self.correct_depth = correct_depth

    async def move_x(step=1):
        await move_tasks.move_x(step=step, parent=self)

    def get_step_size(dist):
        if dist > 4:
            return 1
        else:
            return max(dist-3 + 0.25, 0.25)

    async def sleep(secs):
        duration = rospy.Duration(secs)
        start_time = rospy.Time.now()
        while start_time + duration > rospy.Time.now():
            await Yield()

    rospy.loginfo("Begin sleep")
    await sleep(2)
    rospy.loginfo("End sleep")

    gate_dist = CV().cv_data["gate_red_cw"].coords.x
    # await correct_y(factor=0.5)
    await correct_depth()
    num_corrections = 0
    while gate_dist > 3:
        await move_x(step=get_step_size(gate_dist))
        await yaw_to_cv_object('gate_red_cw', direction=-1, yaw_threshold=math.radians(10),
                               latency_threshold=2, depth_level=0.6, parent=self),
        # await correct_y(factor=(0.5 if num_corrections < 0 else 1))
        await correct_depth()
        await Yield()
        gate_dist = CV().cv_data["gate_red_cw"].coords.x
        rospy.loginfo(f"Gate dist: {gate_dist}")
        num_corrections += 1

    directions = [
        (2, 0, 0),
        (0, 0.2 * direction, 0),
        (2, 0, 0),
        (1, 0, 0)
    ]

    await move_tasks.move_with_directions(directions, correct_yaw=False, correct_depth=True, parent=self)

    rospy.loginfo("Moved through gate")


@task
async def yaw_to_cv_object(self: Task, cv_object: str, direction=1,
                           yaw_threshold=math.radians(30), latency_threshold=10,
                           depth_level=0.5) -> Task[None, None, None]:
    """
    Corrects the yaw relative to the CV object
    """
    DEPTH_LEVEL = State().orig_depth - depth_level
    MAXIMUM_YAW = math.radians(20)

    rospy.loginfo("Starting yaw_to_cv_object")

    async def sleep(secs):
        duration = rospy.Duration(secs)
        start_time = rospy.Time.now()
        while start_time + duration > rospy.Time.now():
            await Yield()

    async def correct_depth():
        # await move_tasks.depth_correction(DEPTH_LEVEL, parent=self)
        await move_tasks.depth_correction(desired_depth=DEPTH_LEVEL, parent=self)

    def is_receiving_cv_data():
        return cv_object in CV().cv_data and \
                rospy.Time.now().secs - CV().cv_data[cv_object].header.stamp.secs < latency_threshold

    def get_step_size(desired_yaw):
        # desired yaw in radians
        return min(abs(desired_yaw), MAXIMUM_YAW)

    async def yaw_until_object_detection():
        while not is_receiving_cv_data():
            rospy.loginfo(f"No {cv_object} detection, setting yaw setpoint {MAXIMUM_YAW}")
            await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, 0, 0, MAXIMUM_YAW * direction),
                                                parent=self)
            await correct_depth()
            await Yield()

    # Yaw until object detection
    await correct_depth()
    await yaw_until_object_detection()

    rospy.loginfo(f"{cv_object} detected. Now centering {cv_object} in frame...")

    # Center detected object in camera frame
    cv_object_yaw = CV().cv_data[cv_object].yaw
    await correct_depth()
    rospy.loginfo('abs(cv_object_yaw)='+str(abs(cv_object_yaw)))
    rospy.loginfo('yaw_threshold='+str(yaw_threshold))
    while abs(cv_object_yaw) > yaw_threshold:
        sign_cv_object_yaw = np.sign(cv_object_yaw)
        correction = get_step_size(cv_object_yaw)
        desired_yaw = sign_cv_object_yaw * correction

        rospy.loginfo(f"Detected yaw {cv_object_yaw} is greater than threshold {yaw_threshold}. Yawing: {desired_yaw}")
        await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, 0, 0, desired_yaw),
                                            parent=self)
        await correct_depth()
        await Yield()

        if (not is_receiving_cv_data()):
            rospy.loginfo(f"{cv_object} detection lost, running yaw_until_object_detection()")
            await yaw_until_object_detection()

        cv_object_yaw = CV().cv_data[cv_object].yaw

    rospy.loginfo(f"{cv_object} centered.")

    await correct_depth()


@task
async def align_path_marker(self: Task, direction=1) -> Task[None, None, None]:
    """
    Corrects the yaw relative to the CV object
    """
    DEPTH_LEVEL = State().orig_depth - 0.5
    MAXIMUM_YAW = math.radians(30)
    YAW_THRESHOLD = math.radians(5)
    PIXEL_THRESHOLD = 70

    rospy.loginfo("Starting align path marker")

    async def move_x(step=1):
        await move_tasks.move_x(step=step, parent=self)

    async def move_y(step=1):
        await move_tasks.move_y(step=step, parent=self)

    async def correct_depth():
        await move_tasks.depth_correction(desired_depth=DEPTH_LEVEL, parent=self)

    async def sleep(secs):
        duration = rospy.Duration(secs)
        start_time = rospy.Time.now()
        while start_time + duration > rospy.Time.now():
            await Yield()

    def get_step_size(desired_yaw):
        # desired yaw in rads
        return min(abs(desired_yaw), MAXIMUM_YAW)

    def get_step_mult_factor(dist, threshold):
        if abs(dist) < threshold:
            return 0
        elif dist > threshold:
            return 1
        else:
            return -1

    async def center_path_marker(pixel_threshold, step_size=0.20, x_offset=0, y_offset=0):
        rospy.loginfo(CV().cv_data["path_marker_distance"])
        pixel_x = CV().cv_data["path_marker_distance"].x + x_offset
        pixel_y = CV().cv_data["path_marker_distance"].y + y_offset

        count = 1
        while (max(pixel_x, pixel_y) > pixel_threshold or min(pixel_x, pixel_y) < -pixel_threshold):
            rospy.loginfo(CV().cv_data["path_marker_distance"])

            await move_x(step=step_size * get_step_mult_factor(pixel_x, pixel_threshold))
            await move_y(step=step_size * get_step_mult_factor(pixel_y, pixel_threshold))

            pixel_x = CV().cv_data["path_marker_distance"].x + x_offset
            pixel_y = CV().cv_data["path_marker_distance"].y + y_offset

            if count % 3 == 0:
                rospy.loginfo("Correcting depth")
                await correct_depth()

            await Yield()

            rospy.loginfo(f"x: {pixel_x}, y: {pixel_y}")

            count += 1

        rospy.loginfo("Finished centering path marker")
        rospy.loginfo(f"x: {pixel_x}, y: {pixel_y}")

    rospy.loginfo("Now aligning path marker in frame...")

    # Center detected object in camera frame
    path_marker_yaw = CV().cv_data["path_marker"].yaw
    await correct_depth()
    rospy.loginfo(f"abs(path_marker_yaw) = '{abs(path_marker_yaw)}")
    rospy.loginfo(f"yaw_threshold = {str(YAW_THRESHOLD)}")

    while abs(path_marker_yaw) > YAW_THRESHOLD:
        sign_path_marker_yaw = np.sign(path_marker_yaw)
        correction = get_step_size(path_marker_yaw)
        desired_yaw = sign_path_marker_yaw * correction

        rospy.loginfo(f"Detected yaw {path_marker_yaw} is greater than threshold {YAW_THRESHOLD}. Yawing: {desired_yaw}"
                      )
        await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, 0, 0, desired_yaw),
                                            parent=self)
        await correct_depth()
        await center_path_marker(pixel_threshold=PIXEL_THRESHOLD)
        await correct_depth()

        await Yield()

        path_marker_yaw = CV().cv_data["path_marker"].yaw

    rospy.loginfo("Path marker centered.")

    await correct_depth()


@task
async def center_path_marker(self: Task):
    DEPTH_LEVEL = State().orig_depth - 0.5
    PIXEL_THRESHOLD = 70

    async def correct_depth(desired_depth=DEPTH_LEVEL):
        await move_tasks.correct_depth(desired_depth=desired_depth, parent=self)
    self.correct_depth = correct_depth

    async def move_x(step=1):
        await move_tasks.move_x(step=step, parent=self)

    async def move_y(step=1):
        await move_tasks.move_y(step=step, parent=self)

    def get_step_mult_factor(dist, threshold):
        if abs(dist) < threshold:
            return 0
        elif dist > threshold:
            return 1
        else:
            return -1

    async def center_path_marker(pixel_threshold, step_size=0.20, x_offset=0, y_offset=0):
        rospy.loginfo(CV().cv_data["path_marker_distance"])
        pixel_x = CV().cv_data["path_marker_distance"].x + x_offset
        pixel_y = CV().cv_data["path_marker_distance"].y + y_offset

        count = 1
        while (max(pixel_x, pixel_y) > pixel_threshold or min(pixel_x, pixel_y) < -pixel_threshold):
            rospy.loginfo(CV().cv_data["path_marker_distance"])

            await move_x(step=step_size * get_step_mult_factor(pixel_x, pixel_threshold))
            await move_y(step=step_size * get_step_mult_factor(pixel_y, pixel_threshold))

            pixel_x = CV().cv_data["path_marker_distance"].x + x_offset
            pixel_y = CV().cv_data["path_marker_distance"].y + y_offset

            if count % 3 == 0:
                rospy.loginfo("Correcting depth")
                await correct_depth()

            await Yield()

            rospy.loginfo(f"x: {pixel_x}, y: {pixel_y}")

            count += 1

        rospy.loginfo("Finished centering path marker")
        rospy.loginfo(f"x: {pixel_x}, y: {pixel_y}")

    await correct_depth()
    await center_path_marker(pixel_threshold=PIXEL_THRESHOLD, x_offset=-120, y_offset=-120)


@task
async def path_marker_to_pink_bin(self: Task, maximum_distance: int = 6):
    DEPTH_LEVEL = State().orig_depth - 0.5
    AREA_THRESHOLD = 1000
    LATENCY_THRESHOLD = 1

    rospy.loginfo("Starting path marker to bins")

    async def correct_depth(desired_depth):
        await move_tasks.correct_depth(desired_depth=desired_depth, parent=self)

    def is_receiving_bin_data(bin_object, latest_detection_time):
        if not latest_detection_time or bin_object not in CV().data:
            return False

        width = CV().cv_data[bin_object].width
        height = CV().cv_data[bin_object].height

        return width * height >= AREA_THRESHOLD and \
            rospy.Time.now().to_sec() - CV().cv_data[bin_object].header.stamp.secs < LATENCY_THRESHOLD and \
            abs(CV().cv_data[bin_object].header.stamp.secs - latest_detection_time) < LATENCY_THRESHOLD

    async def move_x(step=1):
        await move_tasks.move_x(step=step, parent=self)

    def stabilize():
        pose_to_hold = copy.deepcopy(State().state.pose.pose)
        Controls().publish_desired_position(pose_to_hold)

    async def sleep(secs):
        duration = rospy.Duration(secs)
        start_time = rospy.Time.now()
        while start_time + duration > rospy.Time.now():
            await Yield()

    async def move_to_bins():
        count = 1
        bin_red_time = None
        bin_blue_time = None

        await move_x(step=1)

        while not is_receiving_bin_data("bin_red", bin_red_time) \
                or not is_receiving_bin_data("bin_blue", bin_blue_time):
            if "bin_red" in CV().cv_data:
                bin_red_time = CV().cv_data["bin_red"].header.stamp.secs
            if "bin_blue" in CV().cv_data:
                bin_blue_time = CV().cv_data["bin_blue"].header.stamp.secs

            await correct_depth(DEPTH_LEVEL)

            is_receiving_red_bin_data = is_receiving_bin_data("bin_red", bin_red_time)
            is_receiving_blue_bin_data = is_receiving_bin_data("bin_blue", bin_blue_time)

            rospy.loginfo(f"Receiving red bin data: {is_receiving_red_bin_data}")
            rospy.loginfo(f"Receiving blue bin data: {is_receiving_blue_bin_data}")

            step = 0.5 if (is_receiving_red_bin_data or is_receiving_blue_bin_data) else 1
            await move_x(step=step)

            await Yield()

            if count >= maximum_distance:
                rospy.loginfo("Bin not spotted, exiting the loop...")
                break

            count += 1

        rospy.loginfo("Reached pink bins, stabilizing...")
        stabilize()
        await sleep(5)

    await move_to_bins()


@task
async def spiral_bin_search(self: Task) -> Task[None, None, None]:
    DEPTH_LEVEL = State().orig_depth - 0.5
    AREA_THRESHOLD = 1000
    LATENCY_THRESHOLD = 1

    class Direction(Enum):
        FORWARD = 1,
        BACK = 2,
        LEFT = 3,
        RIGHT = 4

    def publish_power(direction: Direction):
        power = Twist()

        if direction == Direction.FORWARD:
            power.linear.x = 0.7
        elif direction == Direction.BACK:
            power.linear.x = -0.7
        elif direction == Direction.LEFT:
            power.linear.y = 1.0
        elif direction == Direction.RIGHT:
            power.linear.y = -1.0

        if direction in [Direction.FORWARD, Direction.BACK]:
            Controls().set_axis_control_type(x=ControlTypes.DESIRED_POWER, y=ControlTypes.DESIRED_POSITION)
        elif direction in [Direction.LEFT, Direction.RIGHT]:
            Controls().set_axis_control_type(x=ControlTypes.DESIRED_POSITION, y=ControlTypes.DESIRED_POWER)

        Controls().publish_desired_power(power, set_control_types=False)

    async def move_step(direction: Direction, steps: float):
        pose = geometry_utils.create_pose(0, 0, 0, 0, 0, 0)

        if direction == Direction.FORWARD:
            pose.position.x = steps
        elif direction == Direction.BACK:
            pose.position.x = -steps
        elif direction == Direction.LEFT:
            pose.position.y = steps
        elif direction == Direction.RIGHT:
            pose.position.y = -steps

        await move_tasks.move_to_pose_local(pose, parent=self)

    DIRECTIONS = [
        (Direction.FORWARD, 1),
        (Direction.LEFT, 1),
        (Direction.BACK, 2),
        (Direction.RIGHT, 2),
        (Direction.FORWARD, 3),
        (Direction.LEFT, 3),
        (Direction.BACK, 4),
        (Direction.RIGHT, 4),
        (Direction.FORWARD, 5),
        (Direction.LEFT, 5),
        (Direction.BACK, 6),
        (Direction.RIGHT, 6),
        (Direction.FORWARD, 7),
        (Direction.LEFT, 7),
        (Direction.BACK, 8),
        (Direction.RIGHT, 8),
    ]

    async def correct_depth(desired_depth):
        await move_tasks.correct_depth(desired_depth=desired_depth, parent=self)

    def is_receiving_bin_data(bin_object, latest_detection_time):
        if not latest_detection_time or bin_object not in CV().cv_data:
            return False

        width = CV().cv_data[bin_object].width
        height = CV().cv_data[bin_object].height

        return width * height >= AREA_THRESHOLD and \
            rospy.Time.now().to_sec() - CV().cv_data[bin_object].header.stamp.secs < LATENCY_THRESHOLD and \
            abs(CV().cv_data[bin_object].header.stamp.secs - latest_detection_time) < LATENCY_THRESHOLD

    def stabilize():
        pose_to_hold = copy.deepcopy(State().state.pose.pose)
        Controls().publish_desired_position(pose_to_hold)

    async def sleep(secs):
        duration = rospy.Duration(secs)
        start_time = rospy.Time.now()
        while start_time + duration > rospy.Time.now():
            await Yield()

    async def search_for_bins():
        rospy.loginfo("Searching for red/blue bins...")
        bin_red_time = None
        bin_blue_time = None

        for direction, secs in DIRECTIONS:
            bin_found = False
            secs *= 0.5
            rospy.loginfo(f"Publishing power: {direction, secs}")

            await move_step(direction, secs)

            iterations = secs / 0.1
            for _ in range(int(iterations)):
                if "bin_red" in CV().cv_data:
                    bin_red_time = CV().cv_data["bin_red"].header.stamp.secs
                if "bin_blue" in CV().cv_data:
                    bin_blue_time = CV().cv_data["bin_blue"].header.stamp.secs

                is_receiving_red_bin_data = is_receiving_bin_data("bin_red", bin_red_time)
                is_receiving_blue_bin_data = is_receiving_bin_data("bin_blue", bin_blue_time)

                bin_found = is_receiving_red_bin_data and is_receiving_blue_bin_data

                if bin_found:
                    break

                await sleep(0.1)
                await Yield()

            await correct_depth(DEPTH_LEVEL)

            if bin_found:
                rospy.loginfo("Found bin, terminating...")
                break

        rospy.loginfo(f"Received red bin data: {is_receiving_red_bin_data}")
        rospy.loginfo(f"Received blue bin data: {is_receiving_blue_bin_data}")

        return bin_found

    return (await search_for_bins())


@task
async def bin_task(self: Task) -> Task[None, None, None]:
    """
    Detects and drops markers into the red bin. Requires robot to have submerged 0.7 meters.
    """

    rospy.loginfo("Started bin task")
    START_DEPTH_LEVEL = State().orig_depth - 0.6
    START_PIXEL_THRESHOLD = 70
    MID_DEPTH_LEVEL = State().orig_depth - 1.0
    MID_PIXEL_THRESHOLD = 30

    FRAME_AREA = 480 * 600

    TIMEOUT = rospy.Duration(240)

    start_time = rospy.Time.now()

    drop_marker = rospy.ServiceProxy('servo_control', SetBool)

    async def correct_x(target):
        await cv_tasks.correct_x(prop=target, parent=self)

    async def correct_y(target):
        await cv_tasks.correct_y(prop=target, parent=self)

    async def correct_z():
        pass

    async def correct_depth(desired_depth):
        await move_tasks.correct_depth(desired_depth=desired_depth, parent=self)
    self.correct_depth = correct_depth

    async def correct_yaw():
        yaw_correction = CV().cv_data["bin_angle"]
        rospy.loginfo(f"Yaw correction: {yaw_correction}")
        sign = 1 if yaw_correction > 0.1 else (-1 if yaw_correction < -0.1 else 0)
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(0, 0, 0, 0, 0, yaw_correction + (sign * 0.1)),
            keep_level=True,
            parent=self
        )
        rospy.loginfo("Corrected yaw")
    self.correct_yaw = correct_yaw

    async def correct_roll_and_pitch():
        imu_orientation = State().imu.orientation
        euler_angles = quat2euler([imu_orientation.w, imu_orientation.x, imu_orientation.y, imu_orientation.z])
        roll_correction = -euler_angles[0] * 1.2
        pitch_correction = -euler_angles[1] * 1.2

        rospy.loginfo(f"Roll, pitch correction: {roll_correction, pitch_correction}")
        await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, roll_correction, pitch_correction, 0),
                                            parent=self)

    async def move_x(step=1):
        await move_tasks.move_x(step=step, parent=self)

    async def move_y(step=1):
        await move_tasks.move_y(step=step, parent=self)

    def get_step_mult_factor(dist, threshold):
        if abs(dist) < threshold:
            return 0
        elif dist > threshold:
            return 1
        else:
            return -1

    async def sleep(secs):
        duration = rospy.Duration(secs)
        start_time = rospy.Time.now()
        while start_time + duration > rospy.Time.now():
            await Yield()

    async def track_bin(target, desired_depth, pixel_threshold, step_size=0.20, x_offset=0, y_offset=0):
        rospy.loginfo(CV().cv_data[f"{target}_distance"])
        pixel_x = CV().cv_data[f"{target}_distance"].x + x_offset
        pixel_y = CV().cv_data[f"{target}_distance"].y + y_offset

        width = CV().cv_data["bin_red"].width
        height = CV().cv_data["bin_red"].height

        count = 1
        while (max(pixel_x, pixel_y) > pixel_threshold or min(pixel_x, pixel_y) < -pixel_threshold) \
                and width * height <= 1/3 * FRAME_AREA:
            rospy.loginfo(CV().cv_data[f"{target}_distance"])

            await move_x(step=step_size * get_step_mult_factor(pixel_x, pixel_threshold))
            await move_y(step=step_size * get_step_mult_factor(pixel_y, pixel_threshold))

            width = CV().cv_data["bin_red"].width
            height = CV().cv_data["bin_red"].height

            pixel_x = CV().cv_data[f"{target}_distance"].x + x_offset
            pixel_y = CV().cv_data[f"{target}_distance"].y + y_offset

            if count % 3 == 0:
                rospy.loginfo("correcting depth")
                await correct_depth(desired_depth=desired_depth)
                rospy.loginfo("correcting roll and pitch")
                await correct_roll_and_pitch()

            if width * height >= 1/6 * FRAME_AREA and \
                    abs(pixel_x) < pixel_threshold * 1.75 and abs(pixel_y) < pixel_threshold * 1.75:
                rospy.loginfo(f"Reached area threshold: area = {width * height}")
                break

            if rospy.Time.now() - start_time > TIMEOUT:
                rospy.logwarn("Track bin timed out")
                break

            await Yield()

            rospy.loginfo(f"x: {pixel_x}, y: {pixel_y}, area: {width * height}")

            count += 1

        rospy.loginfo("Finished tracking bin")
        rospy.loginfo(f"x: {pixel_x}, y: {pixel_y}, area: {width * height}")

    await correct_depth(desired_depth=START_DEPTH_LEVEL)
    await track_bin(target="bin_red", desired_depth=START_DEPTH_LEVEL, pixel_threshold=START_PIXEL_THRESHOLD)

    await correct_yaw()

    await correct_depth(desired_depth=MID_DEPTH_LEVEL)
    await track_bin(target="bin_red", desired_depth=MID_DEPTH_LEVEL, pixel_threshold=MID_PIXEL_THRESHOLD,
                    step_size=0.18, y_offset=30, x_offset=25)

    # If both balls loaded on the RIGHT, this is False
    drop_marker(False)
    rospy.loginfo("Dropped first marker")
    await sleep(3)

    drop_marker(True)
    rospy.loginfo("Dropped second marker")
    await sleep(2)

    await correct_depth(desired_depth=START_DEPTH_LEVEL)
    rospy.loginfo(f"Corrected depth to {START_DEPTH_LEVEL}")

    rospy.loginfo("Completed bin task")


@task
async def octagon_task(self: Task, direction: int = 1) -> Task[None, None, None]:
    """
    Detects, move towards the pink bins, then surfaces inside the octagon. Requires robot to have submerged 0.7 meters.
    """
    rospy.loginfo("Starting octagon task")

    DEPTH_LEVEL_AT_BINS = State().orig_depth - 1.0
    DEPTH_LEVEL_ABOVE_BINS = State().orig_depth - 0.6
    LATENCY_THRESHOLD = 2
    CONTOUR_SCORE_THRESHOLD = 1000
    CONTOUR_SCORE_THRESHOLD = 1000

    async def correct_depth(desired_depth):
        await move_tasks.correct_depth(desired_depth=desired_depth, parent=self)

    async def correct_yaw():
        yaw_correction = CV().cv_data["bin_pink_front"].yaw
        rospy.loginfo(f"Yaw correction: {yaw_correction}")
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(0, 0, 0, 0, 0, yaw_correction * 0.7),
            keep_level=True,
            parent=self
        )
        rospy.loginfo("Corrected yaw")
    self.correct_yaw = correct_yaw

    def is_receiving_pink_bin_data(latest_detection_time):
        return latest_detection_time and "bin_pink_bottom" in CV().cv_data and \
            CV().cv_data["bin_pink_bottom"].score >= CONTOUR_SCORE_THRESHOLD and \
            rospy.Time.now().to_sec() - CV().cv_data["bin_pink_bottom"].header.stamp.secs < LATENCY_THRESHOLD and \
            abs(CV().cv_data["bin_pink_bottom"].header.stamp.secs - latest_detection_time) < LATENCY_THRESHOLD

    def publish_power():
        power = Twist()
        power.linear.x = 0.3
        Controls().set_axis_control_type(x=ControlTypes.DESIRED_POWER)
        Controls().publish_desired_power(power, set_control_types=False)

    async def move_x(step=1):
        await move_tasks.move_x(step=step, parent=self)

    def stabilize():
        pose_to_hold = copy.deepcopy(State().state.pose.pose)
        Controls().publish_desired_position(pose_to_hold)

    async def sleep(secs):
        duration = rospy.Duration(secs)
        start_time = rospy.Time.now()
        while start_time + duration > rospy.Time.now():
            await Yield()

    def get_step_size(last_step_size):
        bin_pink_score = CV().cv_data["bin_pink_front"].score
        step = 0
        if bin_pink_score < 200:
            step = 3
        elif bin_pink_score < 1000:
            step = 2
        elif bin_pink_score < 3000:
            step = 1
        else:
            step = 0.75

        return min(step, last_step_size)

    async def move_to_pink_bins():
        count = 1
        latest_detection_time = None
        moved_above = False

        last_step_size = float('inf')
        await move_x(step=1)
        while not is_receiving_pink_bin_data(latest_detection_time) and not moved_above:
            if "bin_pink_bottom" in CV().cv_data:
                latest_detection_time = CV().cv_data["bin_pink_bottom"].header.stamp.secs

            await correct_depth(DEPTH_LEVEL_AT_BINS if not moved_above else DEPTH_LEVEL_ABOVE_BINS)
            if not moved_above:
                await yaw_to_cv_object('bin_pink_front', direction=direction, yaw_threshold=math.radians(15),
                                       depth_level=0.9, parent=Task.MAIN_ID)

            step = get_step_size(last_step_size)
            await move_x(step=step)
            last_step_size = step

            rospy.loginfo(f"Bin pink front score: {CV().cv_data['bin_pink_front'].score}")

            if CV().cv_data["bin_pink_front"].score > 4000 and not moved_above:
                await correct_depth(DEPTH_LEVEL_ABOVE_BINS + 0.1)
                moved_above = True

                rospy.loginfo("Moved above pink bins")

            await Yield()

            count += 1

            rospy.loginfo(f"Receiving pink bin data: {is_receiving_pink_bin_data(latest_detection_time)}")

        if moved_above:
            await move_tasks.move_with_directions([(1.5, 0, 0)], parent=self)
        else:
            rospy.loginfo("Detected bin_pink_bottom")

        rospy.loginfo("Reached pink bins, stabilizing...")
        stabilize()
        await sleep(5)

    await move_to_pink_bins()

    rospy.loginfo("Surfacing...")
    await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, State().orig_depth - State().depth, 0, 0, 0),
                                        timeout=10, parent=self)
    rospy.loginfo("Finished surfacing")
