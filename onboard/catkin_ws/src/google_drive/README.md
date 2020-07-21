# Google Drive ROS package

This package contains a script to download and upload files to Google Drive. Note that you cannot upload/download G Suite files (e.g. Google docs. Google sheets), but can upload/download any other type of file.

## Usage

### First-time setup
To initialize your google account for use with this script, follow steps 6 and 7 of this [tutorial](https://codelabs.developers.google.com/codelabs/gsuite-apis-intro/#5).

At the end of step 7, you should have downloaded a file with a long name in form `client_secret_LONG_HEX_VALUE.json`. Rename this file to `client_secret.json` and move it into the secrets folder of this package.

This setup will only need to be performed once for your account, for subsequent uses, you merely need to download the `client_secret.json` file.

### Starting the service

To start the node, call
```bash
rosrun google_drive file_transfer.py --noauth_local_webserver
```
If this is your first time authenticating to Drive, you will be asked to go to a link to authorize permissions. Follow the instructions printed. Once you have authenticated successfully, you will be able to call the services.

**DELETE `client_secret.json` AND `storage.json` ONCE YOU HAVE FINISHED YOUR DOWNLOAD/UPLOAD. ACCIDENTALLY COMMITTING THESE FILES CAN COMPROMISE YOUR GOOGLE DRIVE.** 

These files will be located in the secrets folder.
### Download a file

After you have started the node, you may use
```bash
rosservice call /file_download "{file_id: '<file_id>', filepath: '<filepath>'}"
```
Here, `<file_id>` is google drive id of the file you wish to download. The `<filepath>` can either be the package url or the absolute filepath of the location of where to save the file.

Note: to get the `<file_id>`, you can look at the url in Drive, which will look like `https://drive.google.com/file/d/<file_id>/view`. You may need to open the file in a new windows by clicking the three dots in the upper right corner and clicking 'open in a new window'. You can also look at the shareable link.

### Upload a file
After you have started the node, you may use
```bash
rosservice call /file_upload "{folder_id: '<folder_id>', drive_filename: '<drive_filename>', filepath: '<filepath>'}"
```
Here, `<folder_id>` is google drive id of the parent folder under which to upload the file to. The `<drive_filename>` is the name the you want the file to have in Google Drive. The `<filepath>` can either be the package url or the absolute filepath of the file you wish to upload.

Note: All uploads must be placed in a pre-existing folder on Drive, you cannot upload to the root directory of your Drive. To get the `<folder_id>`, you can look at the url in Drive, which will look like `https://drive.google.com/drive/folders/<folder_id>`.
