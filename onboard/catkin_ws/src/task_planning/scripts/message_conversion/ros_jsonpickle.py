import jsonpickle
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
