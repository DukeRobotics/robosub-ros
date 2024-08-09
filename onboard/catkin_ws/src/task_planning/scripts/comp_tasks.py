import rospy
import math
import copy
import time
import numpy as np

from numpy import sign

from transforms3d.euler import quat2euler

from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
from custom_msgs.msg import ControlTypes

from task import Task, task
import move_tasks
import cv_tasks
from utils import geometry_utils
from task import Yield

from interface.state import State
from interface.cv import CV
from interface.controls import Controls

from utils.coroutine_utils import sleep


RECT_HEIGHT_METERS = 0.3048


@task
async def gate_style_task(self: Task) -> Task[None, None, None]:
    """
    Complete two full barrel rolls.
    """

    rospy.loginfo("Started gate style task")

    DEPTH_LEVEL = State().orig_depth - 0.6

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
        # y = -(CV().cv_data["buoy_properties"]["y"])
        # await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, y, 0, 0, 0, 0), parent=self)
        # rospy.loginfo(f"Corrected y {y}")
        await cv_tasks.correct_y("buoy", parent=self)

    async def correct_z():
        # z = -(CV().cv_data["buoy_properties"]["z"])
        # await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, z, 0, 0, 0), parent=self)
        # rospy.loginfo(f"Corrected z {z}")
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
        # await correct_z()
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

        # await correct_depth()
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
                # (0, 0.5, 0),
                (1, 0, 0),
                # (0.5, 0, 0),
            ]
            await move_with_directions(directions, correct_yaw=False)
            await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, 0, 0, -math.radians(90)), parent=self)
            rospy.loginfo("Yaw 90 deg")
            # await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, 0, 0, -math.radians(45)), parent=self)
            # rospy.loginfo("Yaw 45 deg")
            # await yaw_to_cv_object('buoy', direction=-1, yaw_threshold=math.radians(15), depth_level=State().depth, parent=self)
            await move_away_from_buoy()

    else:
        # Circumnavigate buoy
        # directions = [
        #     (0, 1, 0),
        #     # (0, 0.5, 0),
        #     (1, 0, 0),
        #     (1, 0, 0),
        #     (0.5, 0, 0),
        #     (0, -1, 0),
        #     (0, -1, 0),
        #     # (0, -0.5, 0),
        #     (-1, 0, 0),
        #     (-1, 0, 0),
        #     (-0.5, 0, 0),
        #     (0, 1, 0),
        #     # (0, 0.5, 0)
        # ]
        directions = [
            (0, 1.25, 0),
            (2.25, 0, 0),
            (0, -2.5, 0),
            (-2.5, 0, 0),
            (0, 1.25, 0),
        ]
        await move_with_directions(directions, correct_yaw=False)

        # await buoy_circumnavigation_power(depth=depth, parent=self)

        await move_to_buoy()

    # Link up with path marker
    # directions = [
    #     # (0, -1, 0),
    #     # (0, 0.5, 0),
    #     # (1, 0, 0),
    #     # (1, 0, 0),
    #     # (0.5, 0, 0),
    #     # (0, -1, 0),
    #     # (0, -0.5, 0)
    # ]
    # await move_with_directions(directions)


@task
async def buoy_to_octagon(self: Task):
    DEPTH_LEVEL = State().orig_depth - 0.6

    async def move_with_directions(directions):
        await move_tasks.move_with_directions(directions, correct_yaw=False, correct_depth=True, parent=self)

    async def correct_depth():
        await move_tasks.correct_depth(desired_depth=DEPTH_LEVEL, parent=self)
    self.correct_depth = correct_depth

    # Move towards octagon
    directions = [
        (0, -2, 0),
        (0, -2, 0),
        (0, -2, 0),
        (1, 0, 0)
    ]
    await move_with_directions(directions)

    # directions = [
    #     (0, 0, 0, 0, 0, -math.radians(75)),
    # ]
    # await move_with_directions(directions)


@task
async def buoy_circumnavigation_power(self: Task, depth=0.7) -> Task[None, None, None]:

    DEPTH_LEVEL = State().orig_depth - depth

    def publish_power():
        power = Twist()
        power.linear.y = 0.9
        power.angular.z = -0.1
        Controls().set_axis_control_type(x=ControlTypes.DESIRED_POWER, y=ControlTypes.DESIRED_POWER, yaw=ControlTypes.DESIRED_POWER)
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
        await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 1, 0, 0, 0, 0),parent=self)


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


@task
async def coin_flip(self: Task) -> Task[None, None, None]:
    rospy.loginfo("Started coin flip")
    DEPTH_LEVEL = State().orig_depth - 0.6

    def get_yaw_correction():
        orig_imu_orientation = copy.deepcopy(State().orig_imu.orientation)
        orig_imu_euler_angles = quat2euler(geometry_utils.geometry_quat_to_transforms3d_quat(orig_imu_orientation))

        cur_imu_orientation = copy.deepcopy(State().imu.orientation)
        cur_imu_euler_angles = quat2euler(geometry_utils.geometry_quat_to_transforms3d_quat(cur_imu_orientation))

        return orig_imu_euler_angles[2] - cur_imu_euler_angles[2]

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
async def gate_task(self: Task, offset: int) -> Task[None, None, None]:
    rospy.loginfo("Started gate task")
    DEPTH_LEVEL = State().orig_depth - 0.6

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
    await correct_y(factor=0.5)
    await correct_depth()
    num_corrections = 0
    while gate_dist > 3:
        await move_x(step=get_step_size(gate_dist))
        await correct_y(factor=(0.5 if num_corrections < 0 else 1))
        await correct_depth()
        await Yield()
        gate_dist = CV().cv_data["gate_red_cw"].coords.x
        rospy.loginfo(f"Gate dist: {gate_dist}")
        num_corrections += 1

    directions = [
        (2, 0, 0),
        (2, 0, 0),
        (1, 0, 0)
    ]

    await move_tasks.move_with_directions(directions, correct_yaw=False, correct_depth=True, parent=self)

    rospy.loginfo("Moved through gate")


@task
async def yaw_to_cv_object(self: Task, cv_object: str, direction=1,
                           yaw_threshold=math.radians(30), latency_threshold=10, depth_level=0.5) -> Task[None, None, None]:
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
            int(time.time()) - CV().cv_data[cv_object].header.stamp.secs < latency_threshold

    def get_step_size(desired_yaw):
        # desired yaw in rads
        return min(abs(desired_yaw), MAXIMUM_YAW)

    # Yaw until object detection
    await correct_depth()

    while not is_receiving_cv_data():
        await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, 0, 0, MAXIMUM_YAW * direction),
                                            parent=self)
        await Yield()
        rospy.loginfo(f"No buoy detection, setting yaw setpoint {MAXIMUM_YAW}")

    rospy.loginfo(f"{cv_object} detected. Now centering {cv_object} in frame...")

    # Center detected object in camera frame
    cv_object_yaw = CV().cv_data[cv_object].yaw
    await correct_depth()
    while abs(cv_object_yaw) > yaw_threshold:
        sign_cv_object_yaw = sign(cv_object_yaw)
        correction = get_step_size(cv_object_yaw)
        desired_yaw = sign_cv_object_yaw * correction

        rospy.loginfo(f"{cv_object} properties: {CV().cv_data[cv_object]}")
        rospy.loginfo(f"detected yaw {cv_object_yaw} is greater than threshold {yaw_threshold}... Yawing: {desired_yaw}")
        await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, 0, 0, desired_yaw),
                                            parent=self)
        await Yield()

        cv_object_yaw = CV().cv_data[cv_object].yaw

    rospy.loginfo(f"{cv_object} centered.")

    await correct_depth()


@task
async def align_path_marker(self: Task) -> Task[None, None, None]:
    """
    Aligns with the orange path marker. Requires a previous path marker detection and the robot to have submerged 0.7 metres.
    """
    DEPTH_LEVEL = State().orig_depth - 0.5
    rospy.loginfo("Starting path marker alignment")

    async def correct_yaw():
        yaw_correction = CV().cv_data["path_marker"].yaw
        if yaw_correction > 0:
            yaw_correction = np.pi / 2 - yaw_correction
        else:
            yaw_correction = -np.pi / 2 - yaw_correction

        rospy.loginfo(f"Yaw correction: {yaw_correction}")
        sign = 1 if yaw_correction > 0.1 else (-1 if yaw_correction < -0.1 else 0)
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(0, 0, 0, 0, 0, yaw_correction * 1.1),
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

    async def correct_depth():
        await move_tasks.depth_correction(desired_depth=DEPTH_LEVEL, parent=self)

    await correct_depth()
    await correct_yaw()
    # await correct_roll_and_pitch()
    # await correct_depth()
    rospy.loginfo(f"Path marker aligned")


@task
async def bin_task(self: Task) -> Task[None, None, None]:
    """
    Detects and drops markers into the red bin. Requires robot to have submerged 0.7 meters.
    """

    rospy.loginfo("Started bin task")
    START_DEPTH_LEVEL = State().orig_depth - 0.5
    START_PIXEL_THRESHOLD = 70
    MID_DEPTH_LEVEL = State().orig_depth - 1.0
    MID_PIXEL_THRESHOLD = 30
    FINAL_DEPTH_LEVEL = State().orig_depth - 1.25
    FINAL_PIXEL_THRESHOLD = 50

    FRAME_AREA = 480 * 600

    TIMEOUT = rospy.Duration(300)

    start_time = rospy.Time.now()

    DropMarker = rospy.ServiceProxy('servo_control', SetBool)

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
        while (max(pixel_x, pixel_y) > pixel_threshold or min(pixel_x, pixel_y) < -pixel_threshold) and width * height <= 1/3 * FRAME_AREA:
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

            if width * height >= 1/6 * FRAME_AREA and abs(pixel_x) < pixel_threshold * 1.75 and abs(pixel_y) < pixel_threshold * 1.75:
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

    # await correct_depth(desired_depth=FINAL_DEPTH_LEVEL)
    # await track_bin(target="bin_red", desired_depth=FINAL_DEPTH_LEVEL, pixel_threshold=FINAL_PIXEL_THRESHOLD, step_size=0.11)

    # await sleep(3)

    # If both balls loaded on the RIGHT, this is False
    DropMarker(False)
    rospy.loginfo("Dropped first marker")
    await sleep(3)

    DropMarker(True)
    rospy.loginfo("Dropped second marker")
    await sleep(2)

    await correct_depth(desired_depth=START_DEPTH_LEVEL)
    rospy.loginfo(f"Corrected depth to {START_DEPTH_LEVEL}")

    rospy.loginfo("Completed bin task")


@task
async def octagon_task(self: Task) -> Task[None, None, None]:
    """
    Detects, move towards the pink bins, then surfaces inside the octagon. Requires robot to have submerged 0.7 meters.
    """
    rospy.loginfo("Starting octagon task")

    DEPTH_LEVEL = State().orig_depth - 0.9
    LATENCY_THRESHOLD = 2
    CONTOUR_SCORE_THRESHOLD = 1000
    CONTOUR_SCORE_THRESHOLD = 1000

    async def correct_depth():
        await move_tasks.correct_depth(desired_depth=DEPTH_LEVEL, parent=self)
    self.correct_depth = correct_depth

    async def correct_yaw():
        yaw_correction = CV().cv_data["bin_pink_front"].yaw
        rospy.loginfo(f"Yaw correction: {yaw_correction}")
        sign = 1 if yaw_correction > 0.1 else (-1 if yaw_correction < -0.1 else 0)
        # await move_tasks.move_to_pose_local(
        #     geometry_utils.create_pose(0, 0, 0, 0, 0, yaw_correction + (sign * 0.1)),
        #     keep_level=True,
        #     parent=self
        # )
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
            int(time.time()) - CV().cv_data["bin_pink_bottom"].header.stamp.secs < LATENCY_THRESHOLD and \
            abs(CV().cv_data["bin_pink_bottom"].header.stamp.secs - latest_detection_time) < LATENCY_THRESHOLD

    def publish_power():
        power = Twist()
        power.linear.x = 0.3
        Controls().set_axis_control_type(x=ControlTypes.DESIRED_POWER, y=ControlTypes.DESIRED_POWER, yaw=ControlTypes.DESIRED_POWER)
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
        count_when_moved_above = 0

        # publish_power()
        last_step_size = float('inf')
        await move_x(step=1)
        while not is_receiving_pink_bin_data(latest_detection_time) and count - count_when_moved_above < 7:
            if "bin_pink_bottom" in CV().cv_data:
                latest_detection_time = CV().cv_data["bin_pink_bottom"].header.stamp.secs

            if count % 2 == 0:
                # rospy.loginfo("Stabilizing...")
                # stabilize()
                # await sleep(1)

                await correct_depth()
                # await correct_yaw()
                if not moved_above:
                    await yaw_to_cv_object('bin_pink_front', direction=-1, yaw_threshold=math.radians(15), depth_level=0.9, parent=Task.MAIN_ID)

                # await sleep(1)

                # publish_power()
                # rospy.loginfo("Published forward power")
                step = get_step_size(last_step_size)
                await move_x(step=step)
                last_step_size = step

            if CV().cv_data["bin_pink_front"].score > 4000 and not moved_above:
                DEPTH_LEVEL = State().orig_depth - 0.6
                await correct_depth()
                moved_above = True
                count_when_moved_above = count
                rospy.loginfo("Moved above pink bins")

            await Yield()
            await sleep(0.1)

            count += 1

        rospy.loginfo("Reached pink bins, stabilizing...")
        stabilize()
        await sleep(5)

    await move_to_pink_bins()

    rospy.loginfo("Surfacing...")
    await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 1.0, 0, 0, 0), timeout=10, parent=self)
    rospy.loginfo("Finished surfacing")
