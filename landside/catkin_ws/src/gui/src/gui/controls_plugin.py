from rqt_gui_py.plugin import Plugin

from gui.controls_widget import ControlsWidget


class ControlsPlugin(Plugin):

    def __init__(self, context):
        super(ControlsPlugin, self).__init__(context)
        self.widget = ControlsWidget()
        self.widget.setObjectName('ControlsWidget')
        self.widget.setWindowTitle('Controls Plugin')
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
