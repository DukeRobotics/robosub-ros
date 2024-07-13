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
Image type that can be used by image_transport
"""

# ==================================================================================================
#                                             C O D E
# ==================================================================================================

class ImageType():
    RGB8  = 'rgb8'
    BGR8  = 'bgr8'
    MONO8 = 'mono8'

    @classmethod
    def get_channel_count(cls, type:str) -> int:
        if type in [cls.RGB8, cls.BGR8]:
            return 3
        if type in [cls.MONO8]:
            return 1
        raise NotImplementedError(f"Cannot get channel count for type{type} !")