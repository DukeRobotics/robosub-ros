import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.srv import SetCameraInfo
from sensor_msgs.srv import SetCameraInfoResponse
import resource_retriever as rr
import yaml


class CameraInfoManager:

    def __init__(self, cname='camera', url='', namespace=''):
        self.cname = cname
        self.file = rr.get_filename(url, use_protocol=False)
        self.camera_info = loadCalibrationFile(self.file, self.cname)

        self.svc = rospy.Service(namespace+'set_camera_info', SetCameraInfo, self.setCameraInfo)

    def getCameraInfo(self):
        return self.camera_info

    def isCalibrated(self):
        return self.camera_info.K[0] != 0.0

    def setCameraInfo(self, req):
        rospy.logdebug('SetCameraInfo received for ' + self.cname)
        self.camera_info = req.camera_info
        res = SetCameraInfoResponse()
        res.success = saveCalibrationFile(req.camera_info, self.file, self.cname)
        if not res.success:
            res.status_message = "Error storing camera calibration."
        return res


def loadCalibrationFile(filename, cname):
    ci = CameraInfo()
    with open(filename) as f:
        calib = yaml.safe_load(f)
        if calib['camera_name'] != cname:
            rospy.logwarn("[" + cname + "] does not match name " + calib['camera_name'] + " in file " + filename)

        # fill in CameraInfo fields
        ci.width = calib['image_width']
        ci.height = calib['image_height']
        ci.distortion_model = calib['distortion_model']
        ci.D = calib['distortion_coefficients']['data']
        ci.K = calib['camera_matrix']['data']
        ci.R = calib['rectification_matrix']['data']
        ci.P = calib['projection_matrix']['data']

    return ci

def saveCalibrationFile(ci, filename, cname):
    calib = {'image_width': ci.width,
             'image_height': ci.height,
             'camera_name': cname,
             'distortion_model': ci.distortion_model,
             'distortion_coefficients': {'data': ci.D},
             'camera_matrix': {'data': ci.K},
             'rectification_matrix': {'data': ci.R},
             'projection_matrix': {'data': ci.P}}

    with open(filename, 'w') as f:
        yaml.safe_dump(calib, f)
        return True
