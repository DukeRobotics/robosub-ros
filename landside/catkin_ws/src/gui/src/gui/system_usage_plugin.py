from rqt_gui_py.plugin import Plugin

from gui.system_usage_widget import SystemUsageWidget


class SystemUsagePlugin(Plugin):

    def __init__(self, context):
        super(SystemUsagePlugin, self).__init__(context)
        self.widget = SystemUsageWidget()
        self.widget.setObjectName('SystemUsageWidget')
        self.widget.setWindowTitle('System Usage Plugin')
        if context.serial_number() > 1:
            self.widget.setWindowTitle(self.widget.windowTitle() +
                                       (' (%d)' % context.serial_number()))
        context.add_widget(self.widget)

    def shutdown_plugin(self):
        self.widget.close()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
