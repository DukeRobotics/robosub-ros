from rqt_gui_py.plugin import Plugin

from gui.offboard_widget import OffboardWidget

class OffboardPlugin(Plugin):

    def __init__(self, context):
        super(OffboardPlugin, self).__init__(context)
        self.widget = OffboardWidget()
        self.widget.setObjectName('OffboardWidget')
        self.widget.setWindowTitle('Offboard Plugin')
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