from rqt_gui_py.plugin import Plugin

from gui.camera_status_widget import CameraStatusWidget


class CameraStatusPlugin(Plugin):

    def __init__(self, context):
        super(CameraStatusPlugin, self).__init__(context)
        self.widget = CameraStatusWidget()
        self.widget.setObjectName('CameraStatusPlugin')
        self.widget.setWindowTitle('Camera Status Plugin')
        if context.serial_number() > 1:
            self.widget.setWindowTitle(self.widget.windowTitle() +
                                       (' (%d)' % context.serial_number()))
        context.add_widget(self.widget)

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
