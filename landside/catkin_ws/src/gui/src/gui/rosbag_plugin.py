from rqt_gui_py.plugin import Plugin

from gui.rosbag_widget import RosbagWidget


class RosbagPlugin(Plugin):

    def __init__(self, context):
        super(RosbagPlugin, self).__init__(context)
        self.widget = RosbagWidget()
        self.widget.setObjectName('RosbagPlugin')
        self.widget.setWindowTitle('Rosbag Plugin')
        if context.serial_number() > 1:
            self.widget.setWindowTitle(self.widget.windowTitle() +
                                       (' (%d)' % context.serial_number()))
        context.add_widget(self.widget)

    def shutdown_plugin(self):
        self.widget.closeWidget()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        self.widget.default_pkg = instance_settings.value("default_pkg")
        pass
