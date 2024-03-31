from __future__ import annotations

from enum import IntEnum
import inspect
import jsonpickle
from typing import Any, Callable, Coroutine, Generator, Generic, Optional, Type, TypeVar, Union

import rospy
from custom_msgs.msg import TaskUpdate

from message_conversion.jsonpickle_custom_handlers import register_custom_jsonpickle_handlers

# Register all JSONPickle handlers for custom classes
register_custom_jsonpickle_handlers()


class TaskStatus(IntEnum):
    """
    An enum to represent the status of a task. All values are equivalent to the constants defined in
    custom_msgs/TaskUpdate.

    Attributes:
        INITIALIZED: The task has been initialized
        PAUSED: The task has yielded and paused execution
        RESUMED: The task has been sent a value and resumed execution
        THREW: An exception has been sent into and raised in the task and it has resumed execution
        RETURNED: The task has completed by returning a value
        CLOSED: The task has been closed before returning a value
        DELETED: The task has been deleted by the garbage collector before returning a value
        ERRORED: The task has raised an exception and has been closed
    """

    INITIALIZED = TaskUpdate.INITIALIZED
    PAUSED = TaskUpdate.PAUSED
    RESUMED = TaskUpdate.RESUMED
    THREW = TaskUpdate.THREW
    RETURNED = TaskUpdate.RETURNED
    CLOSED = TaskUpdate.CLOSED
    DELETED = TaskUpdate.DELETED
    ERRORED = TaskUpdate.ERRORED


class TaskUpdatePublisher:
    """
    A singleton class to publish task updates.

    Attributes:
        _instance: The singleton instance of this class. Is a static attribute.
        publisher: The publisher for the task_updates topic
    """

    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(TaskUpdatePublisher, cls).__new__(cls)
            cls._instance.__init__()
        return cls._instance

    def __init__(self):
        self.publisher = rospy.Publisher("/task_planning/updates", TaskUpdate, queue_size=0)

    def publish_update(self, task_id: int, parent_id: int, name: str, status: TaskStatus, data: Any) -> None:
        """
        Publish a message to the task_updates topic.

        Args:
            task_id: The id of the task
            parent_id: The id of the parent task
            name: The name of the task
            status: The current status of the task
            data: The data to publish with the status, which should be encodable to JSON.

        Raises:
            AssertionError: If task_id is not an integer
            AssertionError: If parent_id is not an integer
            AssertionError: If name is not a string
            AssertionError: If the status is not a valid TaskStatus
        """

        # Ensure id, parent_id, and name are of the correct types
        assert isinstance(task_id, int), "Task id must be an int"
        assert isinstance(parent_id, int), "Parent id must be an int"
        assert isinstance(name, str), "Name must be a string"

        # Ensure status is one of the valid statuses
        assert status in TaskStatus, "Invalid task status"

        # Options for jsonpickle encoding
        jsonpickle_options = {
            "unpicklable": True,
            "make_refs": False,
            "warn": True,
            "fail_safe": lambda e: "Failed to encode"  # If an object fails to encode, replace it with this string
        }

        # Encode the data to JSON
        msg_data = ""
        try:
            msg_data = jsonpickle.encode(data, **jsonpickle_options)
        except Exception:
            rospy.warn(f"Task with id {task_id} failed to encode data to JSON when publishing {status.name}: {data}")
            msg_data = jsonpickle.encode(str(data), **jsonpickle_options)

        # Create message header
        header = rospy.Header(stamp=rospy.Time.now())

        # Publish the message
        msg = TaskUpdate(header=header, id=task_id, parent_id=parent_id, name=name, status=status, data=msg_data)
        self.publisher.publish(msg)

    def __del__(self):
        self.publisher.unregister()


YieldType = TypeVar("YieldType")
SendType = TypeVar("SendType")
ReturnType = TypeVar("ReturnType")


class Task(Generic[YieldType, SendType, ReturnType]):
    """
    A wrapper around a coroutine to publish task updates for every action taken on the coroutine. All functions defined
    for native coroutines are also defined by this class, including send, throw, close, and __await__. Also defines
    step, which sends `None` to the coroutine.

    Attributes:
        MAIN_ID: The id of the main task
        id: The id of the task
        parent_id: The id of the parent task
        done: If the coroutine has returned, closed, or deleted
        started: If the coroutine has been sent at least one value
        name: The name of the coroutine
        initialized: If this object has been properly initialized
        args: The positional arguments used to initialize the coroutine
        kwargs: The keyword arguments used to initialize the coroutine
    """

    MAIN_ID = 0

    def __init__(self, coroutine: Callable[..., Coroutine[YieldType, SendType, ReturnType]], *args,
                 parent: Optional[Union[Task, int]] = None, **kwargs):
        """
        Initialize the Task object.

        Args:
            coroutine: The native coroutine to wrap
            args: The positional arguments to pass to the coroutine
            parent: The parent task. Must be a Task or Task.MAIN_ID
            kwargs: The keyword arguments to pass to the coroutine

        Raises:
            ValueError: If the parent is not a Task or Task.MAIN_ID
            AssertionError: If the coroutine is not a native coroutine
        """

        self._initialized = False
        self._id = id(self)
        self._done = False
        self._started = False

        # Ensure parent is a Task or the MAIN_ID
        if parent == Task.MAIN_ID:
            self._parent_id = Task.MAIN_ID
        elif isinstance(parent, Task):
            self._parent_id = parent.id
        else:
            raise ValueError("Task parent must be a Task or MAIN_ID. Instead got " + str(parent))

        # Ensure coroutine is a native coroutine
        assert inspect.iscoroutinefunction(coroutine), "Coroutine must be a native coroutine"

        # Initialize the coroutine
        self._coroutine = coroutine(self, *args, **kwargs)
        self._name = coroutine.__name__
        self._args = args
        self._kwargs = kwargs

        initialized_data = {}
        if args:
            initialized_data["args"] = args
        if kwargs:
            initialized_data["kwargs"] = kwargs
        self._publish_update(TaskStatus.INITIALIZED, initialized_data)
        self._initialized = True

    @property
    def id(self):
        """
        The id of the task
        """
        return self._id

    @property
    def parent_id(self):
        """
        The id of the parent task
        """
        return self._parent_id

    @property
    def done(self):
        """
        If the coroutine has returned, closed, or deleted
        """
        return self._done

    @property
    def started(self):
        """
        If the coroutine has been sent at least one value
        """
        return self._started

    @property
    def name(self):
        """
        The name of the coroutine
        """
        return self._name

    @property
    def initialized(self):
        """
        If this object has been properly initialized
        """
        return self._initialized

    @property
    def args(self):
        """
        The positional arguments used to initialize the coroutine
        """
        return self._args

    @property
    def kwargs(self):
        """
        The keyword arguments used to initialize the coroutine
        """
        return self._kwargs

    def _publish_update(self, status: TaskStatus, data: Any) -> None:
        """
        Publish a message to the task_updates topic.

        Args:
            status: The current status of the task
            data: The data to publish with the status, which should be encodable to JSON.

        Raises:
            AssertionError: If the status is not a valid TaskStatus
        """

        TaskUpdatePublisher().publish_update(self._id, self._parent_id, self._name, status, data)

    def step(self):
        """
        Send a None value to the coroutine
        """
        return self.send(None)

    def send(self, value: SendType) -> Union[YieldType, ReturnType]:
        """
        Send a value to the coroutine.

        Args:
            value: The value to send to the coroutine

        Returns:
            The value yielded or returned by the coroutine

        Raises:
            Type[BaseException]: If the coroutine raises an exception
        """

        self._started = True
        try:
            self._publish_update(TaskStatus.RESUMED, value)
            ret_val = self._coroutine.send(value)
            self._publish_update(TaskStatus.PAUSED, ret_val)
            return ret_val
        except StopIteration as e:
            self._done = True
            self._publish_update(TaskStatus.RETURNED, e.value)
            return e.value
        except BaseException as e:
            self._done = True
            self._publish_update(TaskStatus.ERRORED, e)
            raise e

    def throw(self, error: Type[BaseException]) -> Union[YieldType, ReturnType]:
        """
        Raise an exception in the coroutine.

        Args:
            error: The exception to raise in the coroutine

        Returns:
            The value yielded or returned by the coroutine

        Raises:
            Type[BaseException]: If the coroutine raises an exception
        """

        self._started = True
        try:
            self._publish_update(TaskStatus.THREW, error)
            ret_val = self._coroutine.throw(error)
            self._publish_update(TaskStatus.PAUSED, ret_val)
            return ret_val
        except StopIteration as e:
            self._done = True
            self._publish_update(TaskStatus.RETURNED, e.value)
            return e.value
        except BaseException as e:
            self._done = True
            self._publish_update(TaskStatus.ERRORED, e)
            raise e

    def close(self) -> None:
        """
        Close the coroutine.
        """

        if not self._done:
            try:
                self._publish_update(TaskStatus.CLOSED, None)
                self._coroutine.close()
            except BaseException as e:
                self._publish_update(TaskStatus.ERRORED, e)
                raise e
            finally:
                self._done = True

    def __del__(self) -> None:
        """
        Close the coroutine when the object is deleted.
        """

        if self._initialized and not self._done:
            try:
                self._publish_update(TaskStatus.DELETED, None)
                self._coroutine.close()
            except BaseException as e:
                self._publish_update(TaskStatus.ERRORED, e)
                raise e
            finally:
                self._done = True

    def __await__(self) -> Generator[YieldType, SendType, ReturnType]:
        """
        Allow this object to be used in an await expression. Runs the coroutine from where it last left off.

        Yields:
            The value yielded by the coroutine

        Returns:
            The value returned by the coroutine

        Raises:
            Type[BaseException]: If the coroutine raises an exception
        """

        input = None
        output = None
        while not self._done:
            # Yield output and accept input only if the coroutine has been started
            if self._started:
                input = (yield output)
            output = self.send(input)
        return output


def task(func: Callable[..., Coroutine[YieldType, SendType, ReturnType]]) -> \
        Callable[..., Task[YieldType, SendType, ReturnType]]:
    """
    A decorator to wrap a coroutine within Task.
    """

    def wrapper(*args, **kwargs):
        return Task(func, *args, **kwargs)
    return wrapper


class Yield(Generic[YieldType, SendType]):
    """
    A class to allow coroutines to yield and accept input.
    """

    def __init__(self, value: YieldType = None):
        self.value = value

    def __await__(self) -> Generator[YieldType, SendType, SendType]:
        return (yield self.value)