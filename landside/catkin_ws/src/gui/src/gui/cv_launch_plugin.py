from rqt_gui_py.plugin import Plugin

from gui.cv_launch_widget import CVLaunchWidget


class CVLaunchPlugin(Plugin):

    def __init__(self, context):
        super(CVLaunchPlugin, self).__init__(context)
        self.widget = CVLaunchWidget()
        self.widget.setObjectName('CVLaunchWidget')
        self.widget.setWindowTitle('CV Launch Plugin')
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
