from coroutines import task, Yield

pose_from_elsewhere = 1


def set_pose(pose):
    global pose_from_elsewhere
    pose_from_elsewhere = pose


@task
async def check_position():
    print("check_position called")
    await Yield()
    print("check_position finished")
    return pose_from_elsewhere == 2


@task
async def move_to_pose_global(pose):
    print(f"move_to_pose_global setup for: {pose}")

    check = False
    while not check:
        print(f"move_to_pose_global running for: {pose}")
        check = await check_position()
        print(f"value from check_position: {check}")

    print(f"move_to_pose_global finished for: {pose}")


@task
async def test_yield():
    print(await Yield())


@task
async def no_await():
    return 1


@task
async def yield_loop():
    while True:
        print(await Yield())


@task
async def use_yield_loop():
    await yield_loop()
