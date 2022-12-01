#!/usr/bin/env python3

import numpy as np
import torch
import cv2


def nms(labels, dets, scores, thresh=0.01, conf_thresh=0.7):

    # NMS Algorithm
    # https://github.com/rbgirshick/fast-rcnn/blob/master/lib/utils/nms.py
    # Arguments
    #     labels:      list of strings of the classes the model predicted
    #     dets:        boxes coordinates (format:[y1, x1, y2, x2]) as tensor
    #     scores:      box score tensors (N * 4)
    #     thresh:      IoU threshold (the lower the more restrictive)
    #     conf_thresh: confidence value to cut all post-nms predictions
    # Return
    #     nms_labels:  list of strings of the labels of all predictions
    #     nms_boxes:   2D torch tensor of all the bounding box coords
    #     nms_scores:  1D torch tensor of all bouding box prediction confidence

    if not labels:
        return labels, dets, scores

    # Get all unique labels for the given input
    label_set = list(set(labels))

    # Create new predictions tuple with nms approved boxes
    nms_labels = []
    nms_boxes = []
    nms_scores = []

    # Iterate over each unique predicted class
    for label in label_set:

        # Create a mask to select the corresponding rows for a given class
        label_mask = np.array(labels) == label
        # Current box coords for the given class as a 2D tensor
        c_dets_tensor = dets[label_mask]
        # Current scores for the given class as a 2D tensor
        c_score_tensor = scores[label_mask]

        x1 = c_dets_tensor[:, 0].detach()
        y1 = c_dets_tensor[:, 1].detach()
        x2 = c_dets_tensor[:, 2].detach()
        y2 = c_dets_tensor[:, 3].detach()

        areas = (x2 - x1 + 1) * (y2 - y1 + 1)

        # Get the indices that sort a tensor along a given dimension
        # in ascending order by value
        c_score_tensor_indices = torch.argsort(c_score_tensor)
        inv_idx = torch.arange(c_score_tensor_indices.size(0) - 1, -1, -1).long()
        order = c_score_tensor_indices.index_select(0, inv_idx)
        order = c_score_tensor_indices[inv_idx]

        # Keep stores the indices of c_dets_tensor and c_score_tensor to keep
        keep = []
        while order.size()[0] > 0:

            j = order[0]

            if (c_score_tensor[j] > conf_thresh):
                keep.append(j)

            xx1 = torch.max(x1[j], x1[order[1:]])
            yy1 = torch.max(y1[j], y1[order[1:]])
            xx2 = torch.min(x2[j], x2[order[1:]])
            yy2 = torch.min(y2[j], y2[order[1:]])

            w = torch.max(torch.tensor([0.0], dtype=torch.float64),
                          xx2 - xx1 + 1)
            h = torch.max(torch.tensor([0.0], dtype=torch.float64),
                          yy2 - yy1 + 1)
            inter = w * h

            # IoU = i / (area(a) + area(b) - i)
            iou = inter / (areas[j] + areas[order[1:]] - inter)

            # Get all the indices of the conf values that are over the thresh
            inds = np.where(iou <= thresh)[0]
            order = order[inds + 1]

        for index in keep:
            nms_labels.append(label)
            nms_boxes.append(c_dets_tensor[index])
            nms_scores.append(c_score_tensor[index])

    # Check if nms_boxes is empty; if so, return the labels as None
    if len(nms_boxes) == 0:
        return None, dets, scores

    return nms_labels, torch.stack(nms_boxes), torch.stack(nms_scores)


def frame_norm(frame, bbox):
    """ Normalize bbox locations between frame width/height """
    norm_vals = np.full(len(bbox), frame.shape[0])
    norm_vals[::2] = frame.shape[1]
    return (np.clip(np.array(bbox), 0, 1) * norm_vals).astype(int)


def visualize_detections(frame, detections):
    """ Display frame and nn detections from depthai NN """
    color = (255, 0, 0)
    for detection in detections:
        bbox = frame_norm(frame, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
        cv2.putText(frame, f"{int(detection.confidence * 100)}%", (bbox[0] + 10, bbox[1] + 40), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
        cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
    return frame
