import torch
from std_msgs.msg import Header
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, \
    ObjectHypothesisWithPose
from geometry_msgs.msg import Pose2D


def create_detection_msg(detections: torch.Tensor, header: Header) -> Detection2DArray:
    """
    Run a loop at specific frequency

    Parameters
    ----------
        detections : torch.tensor[num_boxes, 6]
            Each element is [x1, y1, x2, y2, confidence, class_id]
        header : std_msgs.msg.Header
            Image source header to keep origin timestamp
    Return
    ------
        Detection2DArray
    """
    detection_array_msg = Detection2DArray()

    detection_array_msg.header = header
    for detection in detections:
        x1, y1, x2, y2, conf, cls = detection.tolist()
        single_detection_msg = Detection2D()
        single_detection_msg.header = header

        # bbox
        bbox = BoundingBox2D()
        w = int(round(x2 - x1))
        h = int(round(y2 - y1))
        cx = int(round(x1 + w / 2))
        cy = int(round(y1 + h / 2))
        bbox.size_x = w
        bbox.size_y = h

        bbox.center = Pose2D()
        bbox.center.x = cx
        bbox.center.y = cy

        single_detection_msg.bbox = bbox

        # class id & confidence
        obj_hyp = ObjectHypothesisWithPose()
        obj_hyp.id = int(cls)
        obj_hyp.score = conf
        single_detection_msg.results = [obj_hyp]

        assert detection_array_msg.detections is not None

        detection_array_msg.detections.append(single_detection_msg)

    return detection_array_msg