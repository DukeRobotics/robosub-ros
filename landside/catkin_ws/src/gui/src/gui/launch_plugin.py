from rqt_gui_py.plugin import Plugin

from gui.launch_widget import LaunchWidget


class LaunchPlugin(Plugin):

    def __init__(self, context):
        super(LaunchPlugin, self).__init__(context)
        self.widget = LaunchWidget()
        self.widget.setObjectName('LaunchWidget')
        self.widget.setWindowTitle('Launch Plugin')
        if context.serial_number() > 1:
            self.widget.setWindowTitle(self.widget.windowTitle() +
                                       (' (%d)' % context.serial_number()))
        context.add_widget(self.widget)

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        self.widget.default_pkg = instance_settings.value("default_pkg")
        pass
