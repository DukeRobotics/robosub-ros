# Copyright (C) 2023 University of South Brittany, Lab-STICC UMR 6285 All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==================================================================================================
"""
Image transport publisher, automaticly create multiple topic
eg: topic_uri='my_topic' will lead to 'my_topic/image/raw_image' and
'my_topic/image/compressed'.
"""

# ==================================================================================================
#                                          I M P O R T S
# ==================================================================================================

import numpy as np
import rospy
import cv2 as cv

from . import TransportType, ImageType

# ==================================================================================================
#                                             C O D E
# ==================================================================================================

class Publisher():
    def __init__(self, topic_uri: str, image_type: str = ImageType.BGR8, queue_size: int = 3) -> None:
        """
        Initialize a new publisher that will generate all ROS publisher corresponding to each
        TransportType available.

        Parameters
        ----------
            topic_uri : str
                Image topic name (eg: 'camera' will automaticly search for topic named
                'camera/image/raw_image' or 'camera/image/compressed')
            image_type : str (default=ImageType.BGR8)
                Your image type, look at image_transport.Imagetype for more option ('bgr8','rgb8',...)
                Allow the publisher to convert to RGB format.
            queue_size : int (default=3)
                Topic queue size
        Return
        ------
            None
        """
        self._topic_uri  = topic_uri
        self._image_type = image_type
        self._publishers = []
        for type_name in TransportType.get_types():
            transport = TransportType.get(type_name)

            topic_full_uri = f'{topic_uri}/image/{type_name}'
            if type_name == 'image_raw':
                topic_full_uri = f'{topic_uri}/image'

            self._publishers.append((
                transport,
                rospy.Publisher(
                    topic_full_uri,
                    transport.get_message_type(),
                    queue_size=queue_size
                )
            ))

    def publish(self, image: np.ndarray) -> None:
        """
        Publish an image to all listeners if at least one is subscribe

        Parameters
        ----------
            image : np.ndarray
                Raw image
        Return
        ------
            None
        """
        if self._image_type == ImageType.BGR8:
            image = cv.cvtColor(image, cv.COLOR_BGR2RGB)

        for transport, pub in self._publishers:
            if pub.get_num_connections() > 0:
                pub.publish(transport.write_message(image))
