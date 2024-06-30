import rospy
import math
import time

from transforms3d.euler import quat2euler

from geometry_msgs.msg import Twist, Pose

from task import Task, task
import move_tasks
from utils import geometry_utils
from task import Yield

from interface.state import State
from interface.cv import CV
from interface.controls import Controls

RECT_HEIGHT_METERS = 0.3048


@task
async def prequal_task(self: Task) -> Task[None, None, None]:
    """
    Complete the prequalification task by moving to a series of local poses. Returns when the robot is at the final pose
    with zero velocity, within a small tolerance, completing the prequalifiacation task.
    """

    countdown_length = 10
    for i in range(countdown_length):
        rospy.loginfo(f"Starting in {countdown_length - i}")
        start_time = time.time()
        while time.time() - start_time < 1:
            await Yield()
            time.sleep(0.01)

    Controls().call_enable_controls(True)

    DEPTH_LEVEL = -0.5

    await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(0, 0, DEPTH_LEVEL, 0, 0, 0),
            parent=self)
    rospy.loginfo(f"Moved to (0, 0, {DEPTH_LEVEL})")

    DEPTH_LEVEL = State().depth

    async def rotate_deg(angle):
        rad_angle = math.radians(angle)
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(0, 0, 0, 0, 0, rad_angle),
            parent=self)
        rospy.loginfo(f"Rotate {angle}")

    async def track_blue_rectangle(distance, direction):
        rospy.loginfo(f"track_blue_rectangle {distance} {direction}")
        repeats = math.ceil(distance)
        total_dist = 0
        prev_touching_top = False
        prev_touching_bottom = False
        for i in range(repeats):
            step = distance - total_dist if i == repeats-1 else 1
            await move_tasks.move_to_pose_local(
                geometry_utils.create_pose(step * direction, 0, 0, 0, 0, 0),
                parent=self)

            total_dist += step
            rospy.loginfo(f"Moved forward {total_dist}")

            touching_top = CV().cv_data["blue_rectangle_touching_top"]
            touching_bottom = CV().cv_data["blue_rectangle_touching_bottom"]
            if touching_top and not touching_bottom:
                await move_tasks.move_to_pose_local(
                    geometry_utils.create_pose(0, 0.2, 0, 0, 0, 0),
                    parent=self)
                rospy.loginfo("Touching top correction (0, 0.2, 0)")
                if prev_touching_top and not prev_touching_bottom:
                    await rotate_deg(20)
            elif not touching_top and touching_bottom:
                await move_tasks.move_to_pose_local(
                    geometry_utils.create_pose(0, -0.2, 0, 0, 0, 0),
                    parent=self)
                rospy.loginfo("Touching bottom correction (0, -0.2, 0)")
                if not prev_touching_top and prev_touching_bottom:
                    await rotate_deg(-20)

            angle = (CV().cv_data["blue_rectangle_angle"] * -1)
            if abs(angle) > 0:
                rad_angle = math.radians(angle)
                await move_tasks.move_to_pose_local(
                    geometry_utils.create_pose(0, 0, 0, 0, 0, rad_angle),
                    parent=self)
                rospy.loginfo(f"Yaw correction {angle}")

            dist_pixels = CV().cv_data["blue_rectangle_dist"]
            height_pixels = CV().cv_data["blue_rectangle_height"]
            dist_meters = dist_pixels * RECT_HEIGHT_METERS / height_pixels
            if abs(dist_meters) > 0:
                await move_tasks.move_to_pose_local(
                    geometry_utils.create_pose(0, dist_meters, 0, 0, 0, 0),
                    parent=self)
                rospy.loginfo(f"Correction {dist_meters}")

            depth_delta = DEPTH_LEVEL - State().depth
            if abs(depth_delta) > 0:
                await move_tasks.move_to_pose_local(
                    geometry_utils.create_pose(0, 0, depth_delta, 0, 0, 0),
                    parent=self)
                rospy.loginfo(f"Depth correction {depth_delta}")

            prev_touching_top = touching_top
            prev_touching_bottom = touching_bottom

    await track_blue_rectangle(2.5, 1)

    await move_tasks.move_to_pose_local(
        geometry_utils.create_pose(0, 0, -0.3, 0, 0, 0),
        parent=self)
    rospy.loginfo("Moved to (0, 0, -0.3)")
    DEPTH_LEVEL = State().depth

    await track_blue_rectangle(2, 1)

    await move_tasks.move_to_pose_local(
        geometry_utils.create_pose(0, 0, 0.3, 0, 0, 0),
        parent=self)
    rospy.loginfo("Moved to (0, 0, 0.3)")
    DEPTH_LEVEL = State().depth

    await track_blue_rectangle(7, 1)

    directions = [
        (1, 0, 0),
        (1, 0, 0),
        (1, 0, 0),
        (0, 1, 0),
        (0, 1, 0),
        (-1, 0, 0),
        (-1, 0, 0),
        (-1, 0, 0),
        (0, -1, 0),
        (0, -0.8, 0),
    ]
    for direction in directions:
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(direction[0], direction[1], direction[2], 0, 0, 0),
            parent=self)
        rospy.loginfo(f"Moved to {direction}")

    await track_blue_rectangle(7, -1)

    await move_tasks.move_to_pose_local(
        geometry_utils.create_pose(0, 0, -0.2, 0, 0, 0),
        parent=self)
    rospy.loginfo("Moved to (0, 0, -0.2)")
    DEPTH_LEVEL = State().depth

    await track_blue_rectangle(2, -1)

    await move_tasks.move_to_pose_local(
        geometry_utils.create_pose(0, 0, 0.2, 0, 0, 0),
        parent=self)
    rospy.loginfo("Moved to (0, 0, 0.2)")
    DEPTH_LEVEL = State().depth

    await track_blue_rectangle(3, -1)

    Controls().call_enable_controls(False)


@task
async def gate_style_task(self: Task) -> Task[None, None, None]:
    """
    Complete two full barrel rolls.
    """

    init_depth = -0.7
    await move_tasks.move_to_pose_local(
        geometry_utils.create_pose(0, 0, init_depth, 0, 0, 0),
        parent=self)
    rospy.loginfo(f"Moved to (0, 0, {init_depth})")

    DEPTH_LEVEL = State().depth

    rospy.loginfo(f"DEPTH_LEVEL: {DEPTH_LEVEL}")

    async def sleep(secs):
        duration = rospy.Duration(secs)
        start_time = rospy.Time.now()
        while start_time + duration > rospy.Time.now():
            # rospy.loginfo(start_time + duration - rospy.Time.now())
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

        await sleep(5)

        rospy.loginfo("Completed zero")

    async def depth_correction():
        rospy.loginfo(f"State().depth: {State().depth}")
        depth_delta = DEPTH_LEVEL - State().depth
        rospy.loginfo(f"depth_delta: {depth_delta}")

        rospy.loginfo(f"Started depth correction {depth_delta}")
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(0, 0, depth_delta, 0, 0, 0),
            parent=self)
        rospy.loginfo(f"Finished depth correction {depth_delta}")

    await roll()
    State().reset_pose()
    # await depth_correction()
    await roll()
    State().reset_pose()
    await sleep(3)
    await depth_correction()
    await sleep(3)

    imu_orientation = State().imu.orientation
    euler_angles = quat2euler([imu_orientation.w, imu_orientation.x, imu_orientation.y, imu_orientation.z])
    roll_correction = -euler_angles[0]
    pitch_correction = -euler_angles[1]

    rospy.loginfo(f"Roll, pitch correction: {roll_correction, pitch_correction}")
    await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, roll_correction, pitch_correction, 0),
                                        parent=self)
    rospy.loginfo("Reset orientation")


@task
async def buoy_task(self: Task) -> Task[None, None, None]:
    """
    Circumnavigate the buoy.
    """

    init_depth = -0.5
    await move_tasks.move_to_pose_local(
        geometry_utils.create_pose(0, 0, init_depth, 0, 0, 0),
        parent=self)
    rospy.loginfo(f"Moved to (0, 0, {init_depth})")

    DEPTH_LEVEL = State().depth

    async def correct_y():
        y = CV().cv_data["buoy_center_distance"][0]
        await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, -y, 0, 0, 0, 0), parent=self)
        rospy.loginfo(f"Corrected y {y}")

    async def correct_z():
        z = CV().cv_data["buoy_center_distance"][1]
        await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, z, 0, 0, 0), parent=self)
        rospy.loginfo(f"Corrected z {z}")

    async def correct_depth():
        depth_delta = DEPTH_LEVEL - State().depth
        await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, depth_delta, 0, 0, 0), parent=self)
        rospy.loginfo(f"Corrected depth {depth_delta}")

    async def move_x(step=1):
        await move_tasks.move_to_pose_local(geometry_utils.create_pose(step, 0, 0, 0, 0, 0), parent=self)
        rospy.loginfo(f"Moved x {step}")

    while CV().cv_data["buoy_dimensions"][0] < 80:
        await move_x()
        await correct_y()
        await correct_depth()

    directions = [
        (0, -0.5, 0),
        (1, 0, 0),
        (1, 0, 0),
        (0, 1, 0),
        (-1, 0, 0),
        (-1, 0, 0),
        (0, -1, 0),
        (1, 0, 0),
        (1, 0, 0),
        (0, 0.5, 0),
    ]
    for direction in directions:
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(direction[0], direction[1], direction[2], 0, 0, 0),
            parent=self)
        rospy.loginfo(f"Moved to {direction}")