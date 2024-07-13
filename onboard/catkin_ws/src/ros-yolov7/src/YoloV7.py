#!/usr/bin/env python

# ==================================================================================================
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

# ==================================================================================================
#                                          I M P O R T S
# ==================================================================================================

from yolov7.models.experimental import attempt_load
from yolov7.utils.general import non_max_suppression
from yolov7.utils.ros import create_detection_msg
from yolov7.visualizer import draw_detections

import os
import time
from typing import Tuple, Callable

import torch
import cv2
import numpy as np
import rospy

import image_transport
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image

# ==================================================================================================
#                                             C O D E
# ==================================================================================================

class YoloV7:
    def __init__(self):

        rospy.init_node('yolov7')

        self.configs    = self._get_config()
        self.img_buffer = None
        self.model      = attempt_load(self.configs['weights_path'], self.configs['device'])
        self.model.eval()

        image_transport.Subscriber(self.configs['img_topic'], self._image_callback, image_type='rgb8')

        self.pub_detection2D = rospy.Publisher(
            name       = f"{rospy.get_name()}/detection",
            data_class = Detection2DArray,
            queue_size = self.configs['queue_size']
        )

        if self.configs['visualize']:
            self.pub_visualize = image_transport.Publisher(topic_uri=f"{rospy.get_name()}/visualize")

        rospy.loginfo(f"[{rospy.get_name()}] Successfully load of the model start inference ...")

        self._main_loop(
            callback  = self._update,
            frequency = self.configs['frequency']
        )

    def _get_config(self) -> dict:
        """
        Read all ros parameters and create a config dictionary with them.

        Return
        ------
            dict()
                Config dictionary
        """
        configs = {}

        # Read all ros node parameters
        try:
            configs['weights_path'] = str  (rospy.get_param('~weights_path'))
            configs['classes_path'] = str  (rospy.get_param('~classes_path'))
            configs['img_topic']    = str  (rospy.get_param('~img_topic'))
            configs['conf_thresh']  = float(rospy.get_param('~conf_thresh', 0.25))
            configs['iou_thresh']   = float(rospy.get_param('~iou_thresh' , 0.45))
            configs['queue_size']   = int  (rospy.get_param('~queue_size' , 3))
            configs['img_size']     = int  (rospy.get_param('~img_size'   , 640))
            configs['visualize']    = bool (rospy.get_param('~visualize'  , False))
            configs['device']       = str  (rospy.get_param('~device'     , 'cuda'))
            configs['frequency']    = int  (rospy.get_param('~frequency'  , 10))
        except KeyError as e:
            raise Exception(f"[{rospy.get_name()}] Error on init, missing parameter named {e} !")

        # Check if weights file exist
        if not os.path.isfile(configs['weights_path']):
            raise FileExistsError(f"[{rospy.get_name()}] No weights file found at {configs['weights_path']} !")

        # Check if classes file exist
        if not os.path.isfile(configs['classes_path']):
            raise FileExistsError(f"[{rospy.get_name()}] No classes file found at {configs['classes_path']} !")

        # Read classes file and convert it to an array
        with open(configs['classes_path'],'r') as f:
            self.class_labels = f.read().split('\n')

        return configs

    def _main_loop(self, callback:Callable, frequency:int=10):
        """
        Run a loop at specific frequency

        Parameters
        ----------
            frequency : int (default=10)
                The loop frequency in Hz
        Return
        ------
            None
        """
        sleep_time = 1/frequency
        while not rospy.is_shutdown():
            t_start = time.time()

            callback()

            t_duration = time.time() - t_start
            t_sleep    = sleep_time - t_duration
            if t_sleep > 0:
                rospy.sleep(t_sleep)

    def _image_callback(self, image: Image):
        self.img_buffer = image

    def _update(self):
        if self.img_buffer is None:
            return

        channel = 3
        height  = self.img_buffer.height
        width   = self.img_buffer.width
        buffer  = np.frombuffer(self.img_buffer.data,dtype=np.uint8)
        np_img_orig   = np.reshape(buffer,(height, width, channel))

        # handle possible different img formats
        if len(np_img_orig.shape) == 2:
            np_img_orig = np.stack([np_img_orig] * 3, axis=2)

        h_orig, w_orig, _  = np_img_orig.shape
        h_scaled, w_scaled = self.configs['img_size'], self.configs['img_size']

        # automatically resize the image to the next smaller possible size
        np_img_resized = cv2.resize(np_img_orig, (h_scaled, w_scaled))

        # conversion to torch tensor (copied from original yolov7 repo)
        img = np_img_resized.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        img = torch.from_numpy(np.ascontiguousarray(img))
        img = img.float()  # uint8 to fp16/32
        img /= 255  # 0 - 255 to 0.0 - 1.
        img = img.to(self.configs['device'])

        # inference & rescaling the output to original img size
        detections = self._inference(img)
        detections[:, :4] = self._rescale(detections[:, :4], (h_scaled, w_scaled), (h_orig, w_orig))
        detections[:, :4] = detections[:, :4].round()

        # publishing
        detection_msg = create_detection_msg(detections, self.img_buffer.header)
        self.pub_detection2D.publish(detection_msg)

        # visualizing if required
        if self.configs['visualize']:
            bboxes = [[int(x1), int(y1), int(x2), int(y2)]
                      for x1, y1, x2, y2 in detections[:, :4].tolist()]
            classes = [int(c) for c in detections[:, 5].tolist()]
            vis_img = draw_detections(np_img_orig, bboxes, classes,
                                      self.class_labels)
            self.pub_visualize.publish(vis_img)

        self.img_buffer = None

    def _rescale(self, boxes: torch.Tensor, ori_shape: Tuple[int, int],
            target_shape: Tuple[int, int]) -> torch.Tensor:
        """
        Rescale the output to the original image shape

        Parameters
        ----------
            boxes : Union[torch.Tensor, np.ndarray]
                Original bounding boxes as a torch.Tensor or np.array or shape
                [num_boxes, >=4], where the first 4 entries of each element have to be
                [x1, y1, x2, y2].
            ori_shape : Tuple[int, int]
                Original width and height [width, height].
            target_shape
                Target width and height [width, height].
        Return
        ------
            Union[torch.Tensor, np.ndarray]
        """
        xscale = target_shape[1] / ori_shape[1]
        yscale = target_shape[0] / ori_shape[0]

        assert isinstance(boxes,torch.Tensor)

        boxes[:, [0, 2]] *= xscale
        boxes[:, [1, 3]] *= yscale

        return boxes

    @torch.no_grad()
    def _inference(self, image: torch.Tensor) -> torch.Tensor:
        """
        Parameters
        ----------
            image : torch.Tensor
                Source image
            ori_shape : Tuple[int, int]
                Original width and height [width, height].
            target_shape
                Target width and height [width, height].
        Return
        ------
            torch.Tensor
                tensor of shape [num_boxes, 6], where each item is represented as
                [x1, y1, x2, y2, confidence, class_id]
        """
        image = image.unsqueeze(0)
        pred_results = self.model(image)[0]
        detections = non_max_suppression(
            prediction = pred_results,
            conf_thres = self.configs['conf_thresh'],
            iou_thres  = self.configs['iou_thresh']
        )
        if detections:
            detections = detections[0]
        return detections

if __name__ == "__main__":
    YoloV7()
