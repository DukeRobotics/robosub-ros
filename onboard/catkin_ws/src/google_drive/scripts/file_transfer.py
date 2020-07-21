#!/usr/bin/env python

import rospy
from custom_msgs.srv import FileDownload, FileUpload
from google_drive import GoogleDrive
import resource_retriever as rr


class FileTransfer:

    def __init__(self):
        rospy.init_node('file_transfer_server')
        rospy.Service('file_download', FileDownload, self.download_service)
        rospy.Service('file_upload', FileUpload, self.upload_service)
        storage_path = rr.get_filename('package://google_drive/secrets/storage.json', use_protocol=False)
        client_secret_path = rr.get_filename('package://google_drive/secrets/client_secret.json', use_protocol=False)
        self.drive = GoogleDrive(storage_path, client_secret_path)
        rospy.spin()

    def download_service(self, req):
        filepath = rr.get_filename(req.filepath, use_protocol=False)
        self.drive.download(req.file_id, filepath)
        return {'success': True}

    def upload_service(self, req):
        filepath = rr.get_filename(req.filepath, use_protocol=False)
        self.drive.upload(req.drive_filename, req.folder_id, filepath)
        return {'success': True}


if __name__ == '__main__':
    FileTransfer()
