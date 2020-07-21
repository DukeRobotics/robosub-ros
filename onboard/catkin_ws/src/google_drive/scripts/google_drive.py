from googleapiclient import discovery
from googleapiclient.http import MediaIoBaseDownload, MediaFileUpload
from httplib2 import Http
from oauth2client import file, client, tools
import io


class GoogleDrive:

    SCOPES = 'https://www.googleapis.com/auth/drive'

    def __init__(self, storage_path, client_secret_path):
        store = file.Storage(storage_path)
        creds = store.get()
        if not creds or creds.invalid:
            flow = client.flow_from_clientsecrets(client_secret_path, self.SCOPES)
            creds = tools.run_flow(flow, store)
        self.drive = discovery.build('drive', 'v3', http=creds.authorize(Http()))

    def download(self, file_id, filepath):
        request = self.drive.files().get_media(fileId=file_id)
        fh = io.FileIO(filepath, mode='wb')
        downloader = MediaIoBaseDownload(fh, request)
        done = False
        while not done:
            status, done = downloader.next_chunk()
            print("Download %d%%." % int(status.progress() * 100))

    def upload(self, drive_filename, folder_id, filepath):
        file_metadata = {
            'name': drive_filename,
            'parents': [folder_id]
        }
        media = MediaFileUpload(filepath, resumable=True)
        drive_file = self.drive.files().create(body=file_metadata, media_body=media, fields='id').execute()
        print('File ID: %s' % drive_file.get('id'))
