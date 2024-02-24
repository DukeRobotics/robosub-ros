# import rospy

# TODO maybe try to make stepping awaitable instead of requiring you to
# yield independently (in order to yield back down to the root)
class TaskWrapper:
    def __init__(self, coroutine, level):
        self.coroutine = coroutine
        self.done = False
        self.started = False

    def run(self):
        res = None
        while not self.done:
            res = self.step()
        return res

    def step(self):
        return self.send(None)

    def send(self, value):
        if not self.started:
            publish_start()
            self.started = True
        try:
            if value is not None:
                publish_update(value)

            ret_value = self.coroutine.send(value)
            if ret_value is not None:
                publish_yielded_value(ret_value)

            return ret_value
        except StopIteration as e:
            publish_done()
            self.done = True
            return e.value

    def __await__(self):
        return self.coroutine.__await__()


def task(func):
    def wrapper(*args, **kwargs):
        return TaskWrapper(func(*args, **kwargs), kwargs['level'])
    return wrapper


class Yield:
    def __init__(self, value=None):
        self.value = value

    def __await__(self):
        return (yield self.value)


class Transform:
    def __init__(self, task: TaskWrapper, input_transformer=None, output_transformer=None):
        self.task = task
        self.input_transformer = input_transformer
        self.output_transformer = output_transformer

    def __await__(self):
        # First value sent to a newly-started coroutine must be None
        input = None
        output = None
        while not self.task.done:
            # Run the inner task for one step and transform what it gives us
            output = self.task.send(input)
            if self.output_transformer is not None:
                output = self.output_transformer(output)
            # Yield that output outwards, and transform whatever is "send()"ed back inwards
            input = (yield output)
            if self.input_transformer is not None:
                input = self.input_transformer(input)
        return output
