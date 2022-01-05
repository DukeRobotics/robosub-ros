from rqt_gui_py.plugin import Plugin

from gui.system_widget import SystemWidget


class SystemPlugin(Plugin):

    def __init__(self, context):
        super(SystemPlugin, self).__init__(context)
        self.widget = SystemWidget()
        self.widget.setObjectName('SystemWidget')
        self.widget.setWindowTitle('Systems Plugin')
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
