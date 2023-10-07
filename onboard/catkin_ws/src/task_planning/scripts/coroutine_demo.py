class Yield():
    def __await__(self):
        return (yield self)

class TaskWrapper():
    def __init__(self, coroutine):
        self.coroutine = coroutine

    def run(self, value=None):
        self.send(value)
        while True:
            try:
                self.coroutine.send(None)
            except StopIteration as e:
                return e.value

    def step(self):
        try:
            return self.coroutine.send(None)
        except StopIteration as e:
            return e.value
        
    def send(self, value):
        try:
            return self.coroutine.send(value)
        except StopIteration as e:
            return e.value
        
    def __await__(self):
        return self.coroutine.__await__()

def task(func):
    def wrapper(*args, **kwargs):
        return TaskWrapper(func(*args, **kwargs))
    return wrapper

pose_from_elsewhere = 1

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
    