import traceback
from typing import Dict


def exception_to_dict(e: BaseException) -> Dict[str, str]:
    """
    Convert an exception inheriting from BaseException to a dictionary.

    Args:
        e: The exception to convert

    Returns:
        A dictionary with the exception type, message, and traceback
    """
    return {
        "type": type(e).__name__,
        "message": str(e),
        "traceback": traceback.format_exc()
    }


def singleton(cls):
    instances = {}

    def getinstance(*args, **kwargs):
        if cls not in instances:
            instances[cls] = cls(*args, **kwargs)
        return instances[cls]
    return getinstance