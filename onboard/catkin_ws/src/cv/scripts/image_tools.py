import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage


class ImageTools(object):
    def __init__(self):
        self._cv_bridge = CvBridge()

    def convert_ros_msg_to_cv2(self, ros_data, image_encoding='bgr8'):
        """
        Convert from a ROS Image message to a cv2 image.
        """
        try:
            return self._cv_bridge.imgmsg_to_cv2(ros_data, image_encoding)
        except CvBridgeError as e:
            if "[16UC1] is not a color format" in str(e) or "[8UC1] is not a color format" in str(e):
                raise CvBridgeError(
                    "You may be trying to use a Image method (Subscriber, Publisher, conversion) on a depth image " +
                    "message. Original exception: " + str(e))
            raise e

    def convert_ros_compressed_to_cv2(self, compressed_msg):
        np_arr = np.frombuffer(compressed_msg.data, np.uint8)
        return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def convert_ros_compressed_msg_to_ros_msg(self, compressed_msg, encoding='bgr8'):
        cv2_img = self.convert_ros_compressed_to_cv2(compressed_msg)
        ros_img = self._cv_bridge.cv2_to_imgmsg(cv2_img, encoding=encoding)
        ros_img.header = compressed_msg.header
        return ros_img

    def convert_cv2_to_ros_msg(self, cv2_data, image_encoding='bgr8'):
        """
        Convert from a cv2 image to a ROS Image message.
        """
        return self._cv_bridge.cv2_to_imgmsg(cv2_data, image_encoding)

    def convert_cv2_to_ros_compressed_msg(self, cv2_data,
                                          compressed_format='jpg'):
        """
        Convert from cv2 image to ROS CompressedImage.
        """
        return self._cv_bridge.cv2_to_compressed_imgmsg(cv2_data, dst_format=compressed_format)

    def convert_ros_msg_to_ros_compressed_msg(self, image,
                                              image_encoding='bgr8',
                                              compressed_format='jpg'):
        """
        Convert from ROS Image message to ROS CompressedImage.
        """
        cv2_img = self.convert_ros_msg_to_cv2(image, image_encoding)
        cimg_msg = self._cv_bridge.cv2_to_compressed_imgmsg(cv2_img, dst_format=compressed_format)
        cimg_msg.header = image.header
        return cimg_msg

    def convert_to_cv2(self, image, image_encoding='bgr8'):
        """
        Convert any kind of image to cv2.
        """
        cv2_img = None
        if type(image) == np.ndarray:
            cv2_img = image
        elif type(image) == Image:
            cv2_img = self.convert_ros_msg_to_cv2(image, image_encoding=image_encoding)
        elif type(image) == CompressedImage:
            cv2_img = self.convert_ros_compressed_to_cv2(image)
        else:
            raise TypeError("Cannot convert type: " + str(type(image)))
        return cv2_img

    def convert_to_ros_msg(self, image, image_encoding='bgr8'):
        """
        Convert any kind of image to ROS Image.
        """
        ros_msg = None
        if type(image) == np.ndarray:
            ros_msg = self.convert_cv2_to_ros_msg(image, image_encoding=image_encoding)
        elif type(image) == Image:
            ros_msg = image
        elif type(image) == CompressedImage:
            ros_msg = self.convert_ros_compressed_msg_to_ros_msg(image, image_encoding=image_encoding)
        else:
            raise TypeError("Cannot convert type: " + str(type(image)))
        return ros_msg

    def convert_to_ros_compressed_msg(self, image, compressed_format='jpg', image_encoding='bgr8'):
        """
        Convert any kind of image to ROS Compressed Image.
        """
        ros_cmp = None
        if type(image) == np.ndarray:
            ros_cmp = self.convert_cv2_to_ros_compressed_msg(image, compressed_format=compressed_format)
        elif type(image) == Image:
            ros_cmp = self.convert_ros_msg_to_ros_compressed_msg(image, image_encoding=image_encoding,
                                                                 compressed_format=compressed_format)
        elif type(image) == CompressedImage:
            ros_cmp = image
        else:
            raise TypeError("Cannot convert type: " + str(type(image)))
        return ros_cmp

    def convert_cv_bridge_depth_encoding_to_encoding_string(self, cv_bridge_depth_encoding, compressed=False):
        encoding_str = ""
        if cv_bridge_depth_encoding == 'mono16':
            encoding_str = '16UC1'
        elif cv_bridge_depth_encoding == 'mono8':
            encoding_str = '8UC1'
        else:
            raise TypeError("Cannot convert image with encoding: " + cv_bridge_depth_encoding)

        if compressed:
            encoding_str += '; compressedDepth'

        return encoding_str

    def convert_encoding_string_to_cv_bridge_depth_encoding(self, encoding_str):
        cv_bridge_depth_encoding = ""
        if encoding_str == '16UC1' or encoding_str == '16UC1; compressedDepth':
            cv_bridge_depth_encoding = 'mono16'
        elif encoding_str == '8UC1' or encoding_str == '8UC1; compressedDepth':
            cv_bridge_depth_encoding = 'mono8'
        else:
            raise TypeError("Cannot convert image with encoding: " + encoding_str)

        return cv_bridge_depth_encoding

    def convert_depth_to_ros_msg(self, image, image_encoding):
        ros_msg = None
        if type(image) == np.ndarray:
            ros_msg = self.convert_cv2_to_ros_msg(image, image_encoding=image_encoding)
        elif type(image) == Image:
            image.encoding = self.convert_cv_bridge_depth_encoding_to_encoding_string(image_encoding)
            ros_msg = image
        elif type(image) == CompressedImage:
            ros_msg = self.convert_compressedDepth_to_image_msg(image)
        else:
            raise TypeError("Cannot convert type: " + str(type(image)))
        return ros_msg

    def convert_depth_to_ros_compressed_msg(self, image, image_encoding):
        ros_cmp = None
        if type(image) == np.ndarray:
            ros_cmp = self.convert_cv2_to_ros_compressed_msg(image, compressed_format='png')
            ros_cmp.format = self.convert_cv_bridge_depth_encoding_to_encoding_string(image_encoding, compressed=True)

        elif type(image) == Image:
            image.encoding = image_encoding
            ros_cmp = self.convert_ros_msg_to_ros_compressed_msg(image, image_encoding=image_encoding,
                                                                 compressed_format='png')
            ros_cmp.format = self.convert_cv_bridge_depth_encoding_to_encoding_string(image_encoding, compressed=True)

        elif type(image) == CompressedImage:
            ros_cmp = image
        else:
            raise TypeError("Cannot convert type: " + str(type(image)))
        return ros_cmp

    def convert_depth_to_cv2(self, image):
        cv2_img = None
        if type(image) == np.ndarray:
            cv2_img = image
        elif type(image) == Image:
            encoding = image.encoding.split(";")[0]
            cv_encoding = self.convert_encoding_string_to_cv_bridge_depth_encoding(encoding)
            cv2_img = self.convert_ros_msg_to_cv2(image, image_encoding=cv_encoding)
        elif type(image) == CompressedImage:
            cv2_img = self.convert_compressedDepth_to_cv2(image)
        else:
            raise TypeError("Cannot convert type: " + str(type(image)))
        return cv2_img

    def convert_compressedDepth_to_image_msg(self, compressed_image):
        """
        Convert a compressedDepth topic image into a ROS Image message.
        compressed_image must be from a topic /bla/compressedDepth
        as it's encoded in PNG
        Code from: https://answers.ros.org/question/249775/display-compresseddepth-image-python-cv2/
        """
        encoding = compressed_image.format.split(";")[0]
        cv_encoding = self.convert_encoding_string_to_cv_bridge_depth_encoding(encoding)

        depth_img_raw = self.convert_compressedDepth_to_cv2(compressed_image)
        img_msg = self._cv_bridge.cv2_to_imgmsg(depth_img_raw, cv_encoding)
        img_msg.header = compressed_image.header
        img_msg.encoding = encoding
        return img_msg

    def convert_compressedDepth_to_cv2(self, compressed_depth):
        """
        Convert a compressedDepth topic image into a cv2 image.
        compressed_depth must be from a topic /bla/compressedDepth
        as it's encoded in PNG
        Code from: https://answers.ros.org/question/249775/display-compresseddepth-image-python-cv2/
        """
        depth_fmt, compr_type = compressed_depth.format.split(';')
        # remove white space
        depth_fmt = depth_fmt.strip()
        compr_type = compr_type.strip()
        if compr_type != "compressedDepth":
            raise Exception("Compression type is not 'compressedDepth'. You probably subscribed to the wrong topic.")

        # remove header from raw data, if necessary
        if 'PNG' in compressed_depth.data[:12]:
            # If we compressed it with opencv, there is nothing to strip
            depth_header_size = 0
        else:
            # If it comes from a robot/sensor, it has 12 useless bytes apparently
            depth_header_size = 12
        raw_data = compressed_depth.data[depth_header_size:]

        depth_img_raw = cv2.imdecode(np.frombuffer(raw_data, np.uint8),
                                     # the cv2.CV_LOAD_IMAGE_UNCHANGED has been removed
                                     -1)  # cv2.CV_LOAD_IMAGE_UNCHANGED)
        if depth_img_raw is None:
            # probably wrong header size
            raise Exception("Could not decode compressed depth image."
                            "You may need to change 'depth_header_size'!")
        return depth_img_raw
