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
TransportType is the parent class of all Transport protocol, you can take look to TransportType.raw
and TransportType.Compressed to see how to implement it.
"""

# ==================================================================================================
#                                          I M P O R T S
# ==================================================================================================

import importlib.util
import rospy
import numpy
import os
import sys
from typing import List, Any

from . import ImageType
from sensor_msgs.msg import Image

# ==================================================================================================
#                                             C O D E
# ==================================================================================================

class TransportType():

    _instance  = None
    _type_dict = {}

    def __init_subclass__(cls, **kwargs):
        """
        Check if child class declare a variable 'topic_uri'
        """
        for required in ['topic_uri']:
            if not getattr(cls, required):
                raise TypeError(f"Can't instantiate abstract class {cls.__name__} without {required} attribute defined")
        return super().__init_subclass__(**kwargs)

    @classmethod
    def initalize(cls):
        """
        Initialize the TransportType singleton, it will look at all '.py' file inside TransportType
        folder, import then and instantiate objects with the same name as their files.
        """
        if TransportType._instance is not None:
            raise Exception(f"A {cls.__name__} if already instanced !")

        cls._instance = TransportType()

        file_path = os.path.dirname(os.path.abspath(__file__))
        type_names = [file_name[:-3] for file_name in os.listdir(f'{file_path}/TransportType') if file_name.endswith('.py')]

        for type_name in type_names:
            spec = importlib.util.spec_from_file_location(f"module.{type_name}", f"{file_path}/TransportType/{type_name}.py")
            assert spec is not None
            module = importlib.util.module_from_spec(spec)
            sys.modules[f"module.{type_name}"] = module
            assert spec.loader is not None
            spec.loader.exec_module(module)
            class_ = getattr(module, type_name)
            instance = class_()
            cls._type_dict[instance.topic_uri] = instance

    @classmethod
    def get_instance(cls) -> 'TransportType':
        """
        Get the singleton instance of TransportType, if not initialize yet,
        initialize it.

        Return
        ------
            image_transport.TransportType
        """
        if TransportType._instance is None:
            TransportType.initalize()
        assert cls._instance is not None
        return cls._instance

    @classmethod
    def get_types(cls) -> List[str]:
        """
        Get list of all transport type loaded

        Return
        ------
            List[str]
        """
        return list(cls.get_instance()._type_dict.keys())

    @classmethod
    def get(cls, type_name : str) -> 'TransportType':
        """
        Get a specific transport type instance from it's name

        Parameters
        ----------
            type_name : str
                Name of the transport type (eg:'compressed')
        Return
        ------
            image_transport.TransportType
        """
        if type_name in cls.get_instance()._type_dict:
            return cls.get_instance()._type_dict[type_name]
        else:
            return cls.get_instance()._type_dict['image_raw']

    def read_message(self, message : Any, image_type : str = ImageType.BGR8) -> Image:
        """
        Child class function that need to be implemented, it will read an incoming message and
        return it's corresponding image.

        Parameters
        ----------
            message : any
                Message that need to be convert
            image_type : str (default=ImageType.BGR8)
                Image type, look at image_transport.Imagetype for more option ('bgr8','rgb8',...)
        Return
        ------
            sensor_msgs.Image
        """
        raise NotImplementedError()

    def write_message(self, image : numpy.ndarray, image_type : str = ImageType.BGR8) -> Any:
        """
        Child class function that need to be implemented, it will read an incoming image and
        return it's corresponding message according to the transport type.

        Parameters
        ----------
            image : numpy.ndarray
                Numpy image (same as OpenCv::Mat, no need to convert) that need to be convert
        Return
        ------
            any
        """
        raise NotImplementedError()

    def get_message_type(self) -> type:
        """
        Child class function that need to be implemented, it will return the ROS message type of the
        transport type.

        Return
        ------
            type
        """
        raise NotImplementedError()