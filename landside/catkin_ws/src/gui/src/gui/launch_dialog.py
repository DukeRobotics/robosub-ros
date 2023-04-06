import os
import glob

from python_qt_binding import loadUi, QtWidgets, QtGui
from python_qt_binding.QtWidgets import QDialog, QMessageBox, QScrollArea, QFormLayout, QWidget
from python_qt_binding.QtGui import QPalette
from python_qt_binding.QtCore import Qt, QRegularExpression, pyqtSignal, pyqtProperty

import rospy
import resource_retriever as rr

from custom_msgs.srv import StartLaunch

import xml.etree.ElementTree as ET
import json

SCROLLABLE_THRESHOLD = 1


class LaunchDialog(QDialog):

    ROOT_PATH = '/root/dev/robosub-ros/onboard/catkin_ws/src'

    node_launched = pyqtSignal(int, str, str, str, name='nodeLaunched')

    def __init__(self, p):
        super(LaunchDialog, self).__init__()

        ui_file = rr.get_filename('package://gui/resource/LaunchDialog.ui', use_protocol=False)
        loadUi(ui_file, self)

        self.parent = p
        self.default_package = ''

        self.reset()

        self.package_name_box.activated.connect(self.package_name_selected)
        self.node_name_box.activated.connect(self.node_name_selected)
        self.accept_button.clicked.connect(self.click_ok)

        self.collapse_button.clicked.connect(self.collapse)

        self.arg_form_rows = []

    @pyqtProperty(str)
    def default_pkg(self):
        return self.default_package

    @default_pkg.setter
    def default_pkg(self, value):
        self.default_package = value

        # Making self.default_package selected by default
        self.package_name_box.setCurrentText(self.default_package)
        self.setup_node_name_box(self.default_package)
        self.node_name_box.setEnabled(bool(self.default_package))

    def get_package_names(self):
        package_list = glob.glob(os.path.join(self.ROOT_PATH, '*/'))
        return [''] + [str(os.path.split(f[:-1])[1]) for f in package_list]

    def reset(self):
        self.package_name_box.clear()
        self.package_name_box.addItems(self.get_package_names())
        self.package_name_box.setEnabled(True)
        self.node_name_box.clear()
        self.node_name_box.setEnabled(False)
        self.other_args_input.clear()
        self.accept_button.setEnabled(False)

    def get_launchables(self, package_name):
        if package_name:
            package_dir = os.path.join(self.ROOT_PATH, package_name)
            launch_files = glob.glob(os.path.join(package_dir, 'launch', '*.launch'))
            script_files = glob.glob(os.path.join(package_dir, 'scripts', '*.py'))
            executables = [f for f in script_files if os.access(f, os.X_OK)]
            return [''] + launch_files + executables
        else:
            return []

    def package_name_selected(self, item_index):
        self.clear_arg_form_rows()

        if item_index == 0:
            self.node_name_box.setEnabled(False)
            return

        selected_package = self.package_name_box.itemText(item_index)
        self.setup_node_name_box(selected_package)
        self.node_name_box.setEnabled(True)

    def setup_node_name_box(self, selected_package):
        self.node_name_box.clear()
        launchables = self.get_launchables(selected_package)
        launchable_names = [os.path.split(f)[1] for f in launchables]
        self.node_name_box.addItems(launchable_names)

        index_of_first_script_file = None
        for index, launchable_name in enumerate(launchable_names):
            if launchable_name.endswith('.py'):
                index_of_first_script_file = index
                break

        if index_of_first_script_file and (index_of_first_script_file > 1) and \
           (index_of_first_script_file < len(launchable_names)):
            self.node_name_box.insertSeparator(index_of_first_script_file)

    def clear_arg_form_rows(self):
        if len(self.arg_form_rows) > SCROLLABLE_THRESHOLD:
            self.form_layout.removeRow(2)

        else:
            for row in self.arg_form_rows:
                self.form_layout.removeRow(row['label'])

        self.arg_form_rows = []

    def node_name_selected(self, item_index):
        self.clear_arg_form_rows()
        args_form_layout = self.form_layout
        scroll = False

        if item_index == 0:
            self.accept_button.setEnabled(False)
            return
        else:
            self.accept_button.setEnabled(True)

        selected_node = self.node_name_box.currentText()
        selected_node_file_type = selected_node.split(".")[1]

        if selected_node_file_type != "launch":
            return

        # Selected node file is a launch file
        self.get_args_from_launch_file()

        if len(self.arg_form_rows) > SCROLLABLE_THRESHOLD:
            args_form_layout = QFormLayout()
            scroll = True

        for row, arg in enumerate(self.arg_form_rows):
            arg['allow_empty'] = True

            default_value = arg.get('default', '')

            label = QtWidgets.QLabel(arg['name'])

            input = QtWidgets.QLineEdit()
            input.setText(default_value)

            toolTip = ""

            if "value" in arg:
                input.setReadOnly(True)
                input.setText(arg["value"])
                palette = QPalette()
                palette.setColor(QPalette.Base, Qt.lightGray)
                palette.setColor(QPalette.Text, Qt.darkGray)
                input.setPalette(palette)

            if "doc" in arg:
                doc_dict = {}
                try:
                    doc_dict = json.loads(arg["doc"])
                except Exception:
                    rospy.logwarn(f"Could not parse JSON doc string for argument `{arg['name']}` in "
                                  f"`{selected_node}`. Defaulting to unrestricted string input.")

                if "options" in doc_dict:
                    input = self.interpret_options(selected_node, arg, doc_dict, input)

                elif "regex" in doc_dict:
                    input, toolTip = self.interpret_regex(selected_node, arg, doc_dict, input, toolTip)

                elif "type" in doc_dict:
                    input, toolTip = self.interpret_type(selected_node, arg, doc_dict, input, toolTip)

                if "help" in doc_dict:
                    if isinstance(doc_dict["help"], str) and doc_dict["help"].strip():
                        if toolTip:
                            toolTip += " | Help: "
                        toolTip += doc_dict["help"]

                if "allowEmpty" in doc_dict:
                    if isinstance(doc_dict["allowEmpty"], bool):
                        arg['allow_empty'] = doc_dict["allowEmpty"]
                    else:
                        rospy.logwarn(f"The property allowEmpty for argument `{arg['name']}` in `{selected_node}` "
                                      f"is not a valid boolean. Defaulting to allow empty input.")

            if toolTip:
                label.setText(label.text() + " (?)")
                label.setToolTip(toolTip)

            # row inserted at position row + 2, after the Package and Node Name rows
            if not scroll:
                args_form_layout.insertRow(row + 2, label, input)
            else:
                args_form_layout.insertRow(row, label, input)
                # args_form_layout.insertRow(row+1, QtWidgets.QLabel(arg['name']), QtWidgets.QLineEdit())
                # args_form_layout.insertRow(row+2, QtWidgets.QLabel(arg['name']), QtWidgets.QLineEdit())
                # args_form_layout.insertRow(row+3, QtWidgets.QLabel(arg['name']), QtWidgets.QLineEdit())
                # args_form_layout.insertRow(row+4, QtWidgets.QLabel(arg['name']), QtWidgets.QLineEdit())
                # args_form_layout.insertRow(row+5, QtWidgets.QLabel(arg['name']), QtWidgets.QLineEdit())
                # args_form_layout.insertRow(row+6, QtWidgets.QLabel(arg['name']), QtWidgets.QLineEdit())
                # args_form_layout.insertRow(row+7, QtWidgets.QLabel(arg['name']), QtWidgets.QLineEdit())
                # args_form_layout.insertRow(row+8, QtWidgets.QLabel(arg['name']), QtWidgets.QLineEdit())
                # args_form_layout.insertRow(row+9, QtWidgets.QLabel(arg['name']), QtWidgets.QLineEdit())
                # args_form_layout.insertRow(row+10, QtWidgets.QLabel(arg['name']), QtWidgets.QLineEdit())

            arg['label'] = label
            arg['input'] = input

        if scroll:
            widget = QWidget()
            widget.setLayout(args_form_layout)

            scroll_widget = QScrollArea()
            scroll_widget.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
            scroll_widget.setWidgetResizable(True)
            scroll_widget.setWidget(widget)

            self.form_layout.insertRow(2, scroll_widget)

    def get_args_from_launch_file(self):
        package_dir = os.path.join(self.ROOT_PATH, self.package_name_box.currentText())
        launch_file_path = os.path.join(package_dir, 'launch/' + self.node_name_box.currentText())

        tree = ET.parse(launch_file_path)

        def traverse_tree(root):
            for child in root:
                if child.tag == 'arg' and "value" not in child:
                    self.arg_form_rows.append(child.attrib)
                traverse_tree(child)

        root = tree.getroot()
        traverse_tree(root)

    def interpret_options(self, selected_node, arg, doc_dict, input):
        if (isinstance(doc_dict["options"], list) and doc_dict["options"] and
                all(isinstance(e, str) for e in doc_dict["options"])):

            input = QtWidgets.QComboBox()
            input.addItems(doc_dict["options"])

            if "default" in arg:
                if arg["default"] in doc_dict["options"]:
                    input.setCurrentText(arg["default"])
                else:
                    input.insertItem(0, arg["default"])
                    input.setCurrentIndex(0)
                    rospy.loginfo(f"Default value for argument `{arg['name']}` in `{selected_node}` "
                                  f"is not in options list. It has been added as the first option.")

        else:
            rospy.logwarn(f"Options list for argument `{arg['name']}` in `{selected_node}` is not a "
                          f"list of strings. Defaulting to unrestricted string input.")

        return input

    def interpret_regex(self, selected_node, arg, doc_dict, input, toolTip):
        regex = QRegularExpression(doc_dict["regex"])
        if regex.isValid():
            regex_validator = QtGui.QRegularExpressionValidator(regex)
            if "default" in arg:
                if regex.match(arg["default"]).hasMatch() or arg["default"] == "":
                    input.setValidator(regex_validator)
                    toolTip += f"Regex: {doc_dict['regex']}"
                else:
                    rospy.logwarn(f"Default value for argument `{arg['name']}` in `{selected_node}` "
                                  f"does not match regex. Defaulting to unrestricted string input.")
            else:
                input.setValidator(regex_validator)
                toolTip += f"Regex: {doc_dict['regex']}"
        else:
            rospy.logwarn(f"Regex for argument `{arg['name']}` in `{selected_node}` is not valid. "
                          f"Defaulting to unrestricted string input.")

        return (input, toolTip)

    def interpret_type(self, selected_node, arg, doc_dict, input, toolTip):
        add_tooltip = True
        if doc_dict["type"] == "bool":
            if "default" not in arg or arg["default"].lower() == "false":
                input = QtWidgets.QCheckBox()
                input.setChecked(False)
            elif arg["default"].lower() == "true":
                input = QtWidgets.QCheckBox()
                input.setChecked(True)
            else:
                rospy.logwarn(f"Default value for argument `{arg['name']}` in `{selected_node}` is "
                              f"not a valid boolean. Defaulting to unrestricted string input.")
                add_tooltip = False

        elif doc_dict["type"] == "int":
            int_validator = QtGui.QIntValidator()
            if "default" in arg:
                if (arg["default"] == "" or
                        int_validator.validate(arg["default"], 0)[0] == QtGui.QValidator.Acceptable):
                    input.setValidator(int_validator)
                else:
                    rospy.logwarn(f"Default value for argument `{arg['name']}` in `{selected_node}` is "
                                  f"not a valid integer. Defaulting to unrestricted string input.")
                    add_tooltip = False
            else:
                input.setValidator(int_validator)

        elif doc_dict["type"] == "double":
            double_validator = QtGui.QDoubleValidator()
            if "default" in arg:
                if (arg["default"] == "" or
                        double_validator.validate(arg["default"], 0)[0] == QtGui.QValidator.Acceptable):
                    input.setValidator(double_validator)
                else:
                    rospy.logwarn(f"Default value for argument `{arg['name']}` in `{selected_node}` is "
                                  f"not a valid double. Defaulting to unrestricted string input.")
                    add_tooltip = False
            else:
                input.setValidator(double_validator)

        elif doc_dict["type"] == "str":
            pass
        else:
            add_tooltip = False
            rospy.logwarn(f"Type `{doc_dict['type']}` for argument `{arg['name']}` in `{selected_node}`"
                          f" is not valid. Type must be `bool`, `int`, `double`, or `str`. Defaulting "
                          f"to unrestricted string input.")

        if add_tooltip:
            toolTip += f"Type: {doc_dict['type']}"

        return (input, toolTip)

    def click_ok(self):
        package = self.package_name_box.currentText()
        node = self.node_name_box.currentText()
        args = []

        for row in self.arg_form_rows:
            text = ""

            if type(row['input']) is QtWidgets.QComboBox:
                text = row['input'].currentText().strip()
            elif type(row['input']) is QtWidgets.QLineEdit:
                text = row['input'].text().strip()
            elif type(row['input']) is QtWidgets.QCheckBox:
                text = str(row['input'].isChecked())

            if not text and not self.handle_empty_arg_value(row):
                return

            args.append(row['name'] + ":=" + text)

        args.extend(self.other_args_input.text().split(' ') if self.other_args_input.text() != "" else [])

        start_launch = rospy.ServiceProxy('start_node', StartLaunch)
        try:
            resp = start_launch(package, node, args, node.endswith('.launch'))
            self.node_launched.emit(resp.pid, package, node, " ".join(args))
        except rospy.ServiceException as exc:
            rospy.logerr(f'Service did not process request: {str(exc)}')

    def handle_empty_arg_value(self, row):
        if not row['allow_empty'] or "default" not in row:
            self.argument_required_dialog(row['name'])
            return False
        elif row.get('default') != "":
            return self.argument_default_dialog(row['name'], row['default'])
        return True

    def argument_default_dialog(self, default_arg, default_value):
        msg = QMessageBox()

        msg.setIcon(QMessageBox.Information)

        msg.setWindowTitle("Warning")
        msg.setText("Missing argument")
        msg.setInformativeText(f"The value of {default_arg} will be the default value of `{default_value}`."
                               f"Is that ok?")

        msg.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)

        return msg.exec_() == QMessageBox.Ok

    def argument_required_dialog(self, required_arg):
        msg = QMessageBox()

        msg.setIcon(QMessageBox.Warning)

        msg.setWindowTitle("Error")
        msg.setText("Required argument")
        msg.setInformativeText("You must specify a value for " + required_arg)

        msg.setStandardButtons(QMessageBox.Close)

        msg.exec_()

    def collapse(self):
        if self.form.isHidden():
            self.form.show()
            self.accept_button.show()
            self.collapse_button.setArrowType(Qt.UpArrow)
        else:
            self.form.hide()
            self.accept_button.hide()
            self.collapse_button.setArrowType(Qt.DownArrow)

    def reject(self):
        pass
