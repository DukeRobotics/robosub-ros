#!/usr/bin/env python3

import numpy as np
import torch

def nms(labels, dets, scores, thresh=0.01, conf_thresh=0.7):

    # NMS Algorithm
    # Code taken from: https://github.com/rbgirshick/fast-rcnn/blob/master/lib/utils/nms.py
    # Arguments
    #     dets:        boxes coordinates (format:[y1, x1, y2, x2]) as a Nx4 tensor
    #     scores:      box score tensors
    # Return
    #     the index of the selected boxes

    if not labels:
        return labels, dets, torch.tensor(scores)

    label_set = list(set(labels))

    # Create new predictions tuple with nms approved boxes
    nms_labels = []
    nms_boxes = []
    nms_scores = []

    for i in range(len(label_set)):

        label_mask = np.array(labels) == label_set[i]
        c_label_tensor = dets[label_mask]
        print(label_set[i], c_label_tensor)
        c_score_tensor = scores[label_mask]

        x1 = c_label_tensor[:, 0].detach().numpy()
        y1 = c_label_tensor[:, 1].detach().numpy()
        x2 = c_label_tensor[:, 2].detach().numpy()
        y2 = c_label_tensor[:, 3].detach().numpy()

        areas = (x2 - x1 + 1) * (y2 - y1 + 1)
        order = c_score_tensor.argsort()[::-1]

        keep = []
        while order.size > 0:

            j = order[0]

            if (scores[j] > conf_thresh):
                keep.append(j)

            xx1 = np.maximum(x1[j], x1[order[1:]])
            yy1 = np.maximum(y1[j], y1[order[1:]])
            xx2 = np.minimum(x2[j], x2[order[1:]])
            yy2 = np.minimum(y2[j], y2[order[1:]])

            w = np.maximum(0.0, xx2 - xx1 + 1)
            h = np.maximum(0.0, yy2 - yy1 + 1)
            inter = w * h
            ovr = inter / (areas[j] + areas[order[1:]] - inter)

            inds = np.where(ovr <= thresh)[0]
            order = order[inds + 1]

        for index in keep:
            nms_labels.append(label_set[i])
            nms_boxes.append(torch.tensor(c_label_tensor[index]))
            nms_scores.append(torch.tensor(c_score_tensor[index]))

    if len(nms_boxes) == 0:
        return None, dets, torch.tensor(scores) 

    return nms_labels, torch.stack(nms_boxes), torch.stack(nms_scores)