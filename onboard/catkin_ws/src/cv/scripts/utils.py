#!/usr/bin/env python3

import numpy as np
import torch


def nms(labels, dets, scores, thresh=0.01, conf_thresh=0.7):

    # NMS Algorithm
    # https://github.com/rbgirshick/fast-rcnn/blob/master/lib/utils/nms.py
    # Arguments
    #     dets:        boxes coordinates (format:[y1, x1, y2, x2]) as tensor
    #     scores:      box score tensors (N * 4)
    #     thresh:      IoU threshold (the lower the more restrictive)
    #     conf_thresh: confidence value to cut all post-nms predictions
    # Return
    #     nms_labels:  list of strings of the labels of all predictions
    #     nms_boxes:   2D torch tensor of all the bounding box coords
    #     nms_scores:  1D torch tensor of all bouding box prediction confidence

    if not labels:
        return labels, dets, torch.tensor(scores)

    # Get all unique labels for the given input
    label_set = list(set(labels))

    # Create new predictions tuple with nms approved boxes
    nms_labels = []
    nms_boxes = []
    nms_scores = []

    # Iterate over each unique predicted class
    for i in range(len(label_set)):

        # Create a mask to select the corresponding rows for a given class
        label_mask = np.array(labels) == label_set[i]
        # Current box coords for the given class as a 2D tensor
        c_dets_tensor = dets[label_mask]
        # Current scores for the given class as a 2D tensor
        c_score_tensor = scores[label_mask]

        x1 = c_dets_tensor[:, 0].detach().numpy()
        y1 = c_dets_tensor[:, 1].detach().numpy()
        x2 = c_dets_tensor[:, 2].detach().numpy()
        y2 = c_dets_tensor[:, 3].detach().numpy()

        areas = (x2 - x1 + 1) * (y2 - y1 + 1)
        order = c_score_tensor.argsort()[::-1]

        # Keep stores the indices of c_dets_tensor and c_score_tensor to keep
        keep = []
        while order.size > 0:

            j = order[0]

            if (c_score_tensor[j] > conf_thresh):
                keep.append(j)

            xx1 = np.maximum(x1[j], x1[order[1:]])
            yy1 = np.maximum(y1[j], y1[order[1:]])
            xx2 = np.minimum(x2[j], x2[order[1:]])
            yy2 = np.minimum(y2[j], y2[order[1:]])

            w = np.maximum(0.0, xx2 - xx1 + 1)
            h = np.maximum(0.0, yy2 - yy1 + 1)
            inter = w * h

            # IoU = i / (area(a) + area(b) - i)
            iou = inter / (areas[j] + areas[order[1:]] - inter)

            inds = np.where(iou <= thresh)[0]
            order = order[inds + 1]

        for index in keep:
            nms_labels.append(label_set[i])
            nms_boxes.append(torch.tensor(c_dets_tensor[index]))
            nms_scores.append(torch.tensor(c_score_tensor[index]))

    # Check if nms_boxes is empty; if so, return the labels as None
    if len(nms_boxes) == 0:
        return None, dets, torch.tensor(scores)

    return nms_labels, torch.stack(nms_boxes), torch.stack(nms_scores)
