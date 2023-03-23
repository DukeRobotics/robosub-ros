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
        context._handler.help_signal.connect(self.help)

    def help(self, plugin_instance_id_str):
        self.widget.help()

    def shutdown_plugin(self):
        self.widget.close()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        self.widget.hostname = instance_settings.value("hostname")
        self.widget.channel = instance_settings.value("channel")
        pass

    def trigger_configuration(self):
        self.widget.settings()
