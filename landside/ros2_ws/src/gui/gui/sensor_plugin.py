from rqt_gui_py.plugin import Plugin

from gui.sensor_widget import SensorWidget


class SensorPlugin(Plugin):

    def __init__(self, context):
        super(SensorPlugin, self).__init__(context)
        self.widget = SensorWidget()
        self.widget.setObjectName('SensorWidget')
        self.widget.setWindowTitle('Sensor Plugin')
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
