from typing import Callable, Coroutine, Optional, TypeVar

from task import Task, Yield


SendType = TypeVar('SendType')
TransformedSendType = TypeVar('TransformedSendType')
YieldType = TypeVar('YieldType')
TransformedYieldType = TypeVar('TransformedYieldType')
ReturnType = TypeVar('ReturnType')
TransformedReturnType = TypeVar('TransformedReturnType')


async def transform(task: Task[YieldType, TransformedSendType, ReturnType],
                    send_transformer: Optional[Callable[[SendType], TransformedSendType]] = None,
                    yield_transformer: Optional[Callable[[YieldType], TransformedYieldType]] = None,
                    return_transformer: Optional[Callable[[ReturnType], TransformedReturnType]] = None) -> \
                        Coroutine[TransformedYieldType, SendType, TransformedReturnType]:
    """
    Transform the input and output of a task.

    Args:
        task: The task to transform
        send_transformer: A function to transform the values sent into the task. If None, the values are not transformed
        (SendType and TransformedSendType are the same).
        yield_transformer: A function to transform the values yielded by the task. If None, the values are not
        transformed (YieldType and TransformedYieldType are the same).
        return_transformer: A function to transform the value returned by the task. If None, the value is not
        transformed (ReturnType and TransformedReturnType are the same).

    Yields:
        The transformed yielded value of the task

    Returns:
        The transformed returned value of the task
    """

    input = None
    output = None
    while not task.done:
        # Send non-None input only if the task has been started
        if task.started:

            # Yield transformed output and accept input
            input = await Yield(output)

            # Transform the input
            if send_transformer:
                input = send_transformer(input)

        # Send transformed input
        output = task.send(input)

        # If the task is done, output is the return value, so don't transform it with yield_transformer
        # Instead, break the loop and transform the return value
        if task.done:
            break

        # Transform the yielded value
        if yield_transformer:
            output = yield_transformer(output)

    # Transform the return value
    if return_transformer:
        output = return_transformer(output)

    return output
