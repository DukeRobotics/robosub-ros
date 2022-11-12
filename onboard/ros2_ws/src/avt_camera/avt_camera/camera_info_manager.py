from sensor_msgs.msg import CameraInfo
from sensor_msgs.srv import SetCameraInfo
import resource_retriever as rr
import yaml


class CameraInfoManager:

    def __init__(self, node, cname='camera', url='', namespace=''):
        self.node = node
        self.cname = cname
        self.file = rr.get_filename(url, use_protocol=False)
        self.camera_info = self.loadCalibrationFile(self.file, self.cname)

        self.svc = node.create_service(SetCameraInfo, namespace +
                                       'set_camera_info', self.setCameraInfo)

    def getCameraInfo(self):
        return self.camera_info

    def isCalibrated(self):
        return self.camera_info.K[0] != 0.0

    def setCameraInfo(self, req):
        self.node.get_logger().debug('SetCameraInfo received for ' + self.cname)
        self.camera_info = req.camera_info
        res = SetCameraInfo.Response()
        res.success = self.saveCalibrationFile(req.camera_info, self.file, self.cname)
        if not res.success:
            res.status_message = "Error storing camera calibration."
        return res

    def loadCalibrationFile(self, filename, cname):
        ci = CameraInfo()
        with open(filename) as f:
            calib = yaml.safe_load(f)
            if calib['camera_name'] != cname:
                self.node.get_logger().warn("[" + cname + "] does not match name " +
                                            calib['camera_name'] + " in file " + filename)

            # fill in CameraInfo fields
            ci.width = calib['image_width']
            ci.height = calib['image_height']
            ci.distortion_model = calib['distortion_model']
            ci.d = calib['distortion_coefficients']['data']
            ci.k = calib['camera_matrix']['data']
            ci.r = calib['rectification_matrix']['data']
            ci.p = calib['projection_matrix']['data']

        return ci

    def saveCalibrationFile(self, ci, filename, cname):
        calib = {'image_width': ci.width,
                 'image_height': ci.height,
                 'camera_name': cname,
                 'distortion_model': ci.distortion_model,
                 'distortion_coefficients': {'data': ci.d},
                 'camera_matrix': {'data': ci.k},
                 'rectification_matrix': {'data': ci.r},
                 'projection_matrix': {'data': ci.p}}

        with open(filename, 'w') as f:
            yaml.safe_dump(calib, f)
            return True
