import builtins
import jsonpickle
import traceback

import genpy


from message_conversion.ros_message_converter import convert_ros_message_to_dictionary, \
    convert_dictionary_to_ros_message


class ROSMessageHandler(jsonpickle.handlers.BaseHandler):
    """
    JSONPickle handler to convert ROS messages to and from dictionaries
    """
    def flatten(self, obj, data):
        data['ros/type'] = obj._type
        data['ros/data'] = convert_ros_message_to_dictionary(obj)
        return data

    def restore(self, obj):
        message_type = obj['ros/type']
        dictionary = obj['ros/data']
        return convert_dictionary_to_ros_message(message_type, dictionary)


class BaseExceptionHandler(jsonpickle.handlers.BaseHandler):
    """
    JSONPickle handler to convert exceptions to and from dictionaries
    """
    def flatten(self, obj, data):
        data['exception/type'] = type(obj).__name__
        data['exception/message'] = str(obj)
        data['exception/traceback'] = traceback.format_tb(obj.__traceback__)
        return data

    def restore(self, obj):
        exc_type = obj['exception/type']
        exc_message = obj['exception/message']

        # Reconstruct the exception object; does not include traceback
        exc_class = getattr(builtins, exc_type)
        exc_instance = exc_class(exc_message)

        return exc_instance


def register_custom_jsonpickle_handlers():
    """
    Register all custom JSONPickle handlers
    """
    jsonpickle.handlers.register(genpy.Message, ROSMessageHandler, base=True)
    jsonpickle.handlers.register(BaseException, BaseExceptionHandler, base=True)
