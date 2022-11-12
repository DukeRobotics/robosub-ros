#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_msgs.msg import CVObject, SimObjectArray
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_geometry_msgs import do_transform_pose_stamped
from numpy import clip
import resource_retriever as rr
import yaml


class BoundingBox(Node):

    NODE_NAME = "sim_fake_cv_maker"
    VERBOSE_BOUNDING_BOX = False

    def __init__(self):
        super().__init__(self.NODE_NAME)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        config_filepath = rr.get_filename(
            'package://simulation/data/config.yaml',
            use_protocol=False
        )
        with open(config_filepath) as f:
            data = yaml.safe_load(f)
        self.initialize_publishers(data['cv_objects'])
        self.create_subscription(SimObjectArray, "/sim/object_points", self.callback, 10)

    def initialize_publishers(self, cv_object_labels):
        self.pubs = {}
        for label in cv_object_labels:
            self.pubs[label] = self.create_publisher(
                CVObject,
                f'cv/simulation/{label.strip().lower()}/left',
                10)

    def callback(self, data):
        object_data = {}
        for obj in data.objects:
            print(obj.label)
            # average = [0, 0, 0]
            # for pt in obj.points:
            #     transformed = self.point_rel_to_bot(pt)
            #     average[0] += transformed.x
            #     average[1] += transformed.y
            #     average[2] += transformed.z
            # average[0] /= len(obj.points)
            # average[1] /= len(obj.points)
            # average[2] /= len(obj.points)
            # print(obj.label, len(obj.points), average)

            # average = [0, 0, 0]
            # for pt in obj.points:
            #     transformed = self.test_point_rel_to_base_link(pt)
            #     average[0] += transformed.x
            #     average[1] += transformed.y
            #     average[2] += transformed.z
            # average[0] /= len(obj.points)
            # average[1] /= len(obj.points)
            # average[2] /= len(obj.points)
            # print("Base link", obj.label, average)

            object_data[obj.label] = {
                'bounding_box': self.get_bounding_box(obj.points),
                'distance': obj.distance
            }

        for label, info in object_data.items():
            box = CVObject()
            box.score = 1
            class_label = self.get_item_class(label)
            box.label = label
            box.xmin, box.xmax, box.ymin, box.ymax = info['bounding_box']
            box.distance = info['distance']
            if class_label not in self.pubs:
                self.get_logger().error(f"bounding_box.callback: Publisher for label {class_label} not found")
                continue
            if VERBOSE_BOUNDING_BOX:
                self.get_logger().info(f"bounding_box.callback: publishing bounding box {box}")
            self.pubs[class_label].publish(box)

    def get_item_class(self, item_label):
        """
        Returns the type of a sim object given its label in the simulation.
        Example: get_item_class("GateLeftChild#0") = GateLeftChild
        """
        hash_index = item_label.find('#')
        if hash_index < 0:
            return item_label
        return item_label[:hash_index]

    def get_bounding_box(self, points):
        grid_points = [self.get_grid_point(point) for point in points]
        filtered_grid_points = [point for point in grid_points if point != (-1, -1)]
        xs, ys = zip(*(filtered_grid_points if filtered_grid_points else [(-1, -1)]))
        if VERBOSE_BOUNDING_BOX:
            self.get_logger().info(f"bounding_box.get_bounding_box: {xs}, {ys}")
        return clip(min(xs), 0, 1), clip(max(xs), 0, 1), clip(min(ys), 0, 1), clip(max(ys), 0, 1)

    def get_grid_point(self, point):
        print("raw_point", point.x, point.y, point.z)
        rel_point = self.point_rel_to_bot(point)
        # FOV - field of view
        if rel_point.x < 0:
            return (-1, -1)
        print("rel_point", rel_point.x, rel_point.y, rel_point.z)
        # note, since this uses the x value only, the "FOV" isn't completely accurate to a curved camera view
        xFOV = 0.933 * rel_point.x
        yFOV = 0.586 * rel_point.x
        # print(xFOV, yFOV)
        # 0,0 is top-left
        xPix = (xFOV / 2 - rel_point.y) / xFOV
        yPix = (yFOV / 2 - rel_point.z) / yFOV
        # print(xPix, yPix)
        return xPix, yPix  # clip(xPix, 0, 1), clip(yPix, 0, 1)

    def point_rel_to_bot(self, point):
        poses = PoseStamped()
        poses.pose = Pose(position=point, orientation=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0))
        poses.header.frame_id = "odom"
        transform = self.tf_buffer.lookup_transform("cameras_link",
                                                    poses.header.frame_id,
                                                    rclpy.time.Time())
        transformed = do_transform_pose_stamped(poses, transform).pose.position
        return transformed

    def test_point_rel_to_base_link(self, point):
        poses = PoseStamped()
        poses.pose = Pose(position=point, orientation=Quaternion(w=1, x=0, y=0, z=0))
        poses.header.frame_id = "odom"
        transform = self.tf_buffer.lookup_transform("base_link",
                                                    poses.header.frame_id,
                                                    rclpy.time.Time())
        transformed = do_transform_pose_stamped(poses, transform).pose.position
        return transformed


def main(args=None):
    try:
        rclpy.init(args=args)
        bb = BoundingBox()
        rclpy.spin(bb)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        raise
    finally:
        bb.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
