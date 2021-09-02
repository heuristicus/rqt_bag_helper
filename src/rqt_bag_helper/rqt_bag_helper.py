import os
import signal

import rospy
import rospkg
import yaml
import subprocess
import functools

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import (
    QWidget,
    QTreeView,
    QLineEdit,
    QPushButton,
    QLabel,
    QFileDialog,
    QSpinBox,
    QCheckBox,
    QMessageBox,
)
from python_qt_binding.QtGui import QStandardItemModel, QStandardItem
from python_qt_binding.QtCore import QSortFilterProxyModel
import python_qt_binding.QtCore as QtCore


class RqtBagHelper(Plugin):

    BUFFSIZE_DEFAULT = 256
    CHUNKSIZE_DEFAULT = 768

    def __init__(self, context):
        super(RqtBagHelper, self).__init__(context)
        self.setObjectName("rqt_bag_helper")

        # Are we recording a rosbag
        self._recording = False

        # Create QWidget
        self._widget = QWidget()
        ui_file = os.path.join(
            rospkg.RosPack().get_path("rqt_bag_helper"), "resource", "RosBagHelper.ui"
        )
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        self._widget.setObjectName("rqt_bag_helper")
        # Show _widget.windowTitle on left-top of each plugin (when it's set in _widget). This is useful when you
        # open multiple plugins at once. Also if you open multiple instances of your plugin at once, these lines add
        # number to make it easy to tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (" (%d)" % context.serial_number())
            )

        self._model = QStandardItemModel()
        self._topic_tree = self._widget.findChildren(QTreeView, "topicTree")[0]

        self._model.setHorizontalHeaderLabels(["Record", "Rate", "Topic"])
        # There are only two sections so they can't be moved anyway
        self._topic_tree.header().setSectionsMovable(False)
        # self._topic_tree.header().setDefaultSectionSize(200)
        self._topic_tree.setSortingEnabled(True)

        # Set up the model used for sorting and filtering the topics
        self._sort_model = QSortFilterProxyModel()
        self._sort_model.setSourceModel(self._model)
        # Filter on topic column
        self._sort_model.setFilterKeyColumn(2)
        self._topic_tree.setModel(self._sort_model)
        self._widget.findChildren(QLineEdit, "filterEdit")[0].textChanged.connect(
            self._sort_model.setFilterFixedString
        )

        self._widget.findChildren(QPushButton, "loadButton")[0].clicked.connect(
            self._load_file
        )

        self._widget.findChildren(QPushButton, "saveButton")[0].clicked.connect(
            self._save_file
        )

        self._record_button = self._widget.findChildren(QPushButton, "recordButton")[0]
        self._record_button.clicked.connect(self._toggle_record)

        self._output_edit = self._widget.findChildren(QLineEdit, "outputEdit")[0]
        self._prefix_edit = self._widget.findChildren(QLineEdit, "prefixEdit")[0]
        self._regex_edit = self._widget.findChildren(QLineEdit, "regexEdit")[0]
        self._exclude_regex_edit = self._widget.findChildren(
            QLineEdit, "excludeRegexEdit"
        )[0]

        self._limit_spin = self._widget.findChildren(QSpinBox, "limitSpinBox")[0]
        self._max_split_spin = self._widget.findChildren(QSpinBox, "maxSplitSpinBox")[0]
        self._size_spin = self._widget.findChildren(QSpinBox, "sizeSpinBox")[0]
        self._duration_spin = self._widget.findChildren(QSpinBox, "durationSpinBox")[0]
        self._buffer_spin = self._widget.findChildren(QSpinBox, "bufferSpinBox")[0]
        self._chunk_spin = self._widget.findChildren(QSpinBox, "chunkSpinBox")[0]

        self._split_check = self._widget.findChildren(QCheckBox, "splitCheckBox")[0]

        self._lz4_check = self._widget.findChildren(QCheckBox, "lz4CheckBox")[0]
        # Use functools to pass the widget so we can reuse the function
        self._lz4_check.stateChanged.connect(
            functools.partial(self._compression_changed, self._lz4_check)
        )

        self._bz2_check = self._widget.findChildren(QCheckBox, "bz2CheckBox")[0]
        self._bz2_check.stateChanged.connect(
            functools.partial(self._compression_changed, self._bz2_check)
        )

        self._tcp_nodelay_check = self._widget.findChildren(
            QCheckBox, "tcpNodelayCheckBox"
        )[0]
        self._udp_check = self._widget.findChildren(QCheckBox, "udpCheckBox")[0]
        self._repeat_latched_check = self._widget.findChildren(
            QCheckBox, "repeatLatchedCheckBox"
        )[0]
        self._publish_check = self._widget.findChildren(QCheckBox, "publishCheckBox")[0]
        self._advanced_check = self._widget.findChildren(QCheckBox, "advancedCheckBox")[
            0
        ]
        self._advanced_check.stateChanged.connect(self._advanced_changed)

        context.add_widget(self._widget)

    def _add_topic(self, topic_name):
        """
        Add a topic to the tree
        :param topic_name: Name of the topic to add
        :return:
        """
        parent = self._model.invisibleRootItem()

        check_item = QStandardItem()
        check_item.setCheckable(True)
        check_item.setCheckState(QtCore.Qt.Checked)

        rate_item = QStandardItem()
        rate_item.setEditable(False)

        topic_item = QStandardItem(topic_name)
        topic_item.setEditable(False)
        parent.appendRow([check_item, rate_item, topic_item])

    def _set_advanced(self, enabled):
        """
        Enable or disable the advanced settings for buffsize and chunksize
        :param enabled: True to enable, false to disable
        :return:
        """
        self._advanced_check.setCheckState(
            QtCore.Qt.Checked if enabled else QtCore.Qt.Unchecked
        )
        self._chunk_spin.setEnabled(enabled)
        self._buffer_spin.setEnabled(enabled)

    @staticmethod
    def _get_widget_value(widget):
        if type(widget) == QSpinBox:
            return widget.value()
        if type(widget) == QCheckBox:
            return True if widget.checkState() == QtCore.Qt.Checked else False
        if type(widget) == QLineEdit:
            return widget.text()

    def _generate_arg_dict(self):
        """
        Generate a dict of all the arg values, in the format of the yaml

        If the advanced checkbox is unchecked, buffsize and chunksize will not be in the dict. They will also be
        excluded if they are set to the default values

        :return:
        """
        arg_dict = {
            "buffsize": self._get_widget_value(self._buffer_spin),
            "bz2": self._get_widget_value(self._bz2_check),
            "chunksize": self._get_widget_value(self._chunk_spin),
            "duration": self._get_widget_value(self._duration_spin),
            "exclude": self._get_widget_value(self._exclude_regex_edit),
            "limit": self._get_widget_value(self._limit_spin),
            "lz4": self._get_widget_value(self._lz4_check),
            "max-splits": self._get_widget_value(self._max_split_spin),
            "output-name": self._get_widget_value(self._output_edit),
            "output-prefix": self._get_widget_value(self._prefix_edit),
            "publish": self._get_widget_value(self._publish_check),
            "regex": self._get_widget_value(self._regex_edit),
            "repeat-latched": self._get_widget_value(self._repeat_latched_check),
            "size": self._get_widget_value(self._size_spin),
            "split": self._get_widget_value(self._split_check),
            "tcpnodelay": self._get_widget_value(self._tcp_nodelay_check),
            "udp": self._get_widget_value(self._udp_check),
        }
        if (
            self._advanced_check.checkState() is QtCore.Qt.Unchecked
            or arg_dict["buffsize"] == RqtBagHelper.BUFFSIZE_DEFAULT
            or arg_dict["chunksize"] == RqtBagHelper.CHUNKSIZE_DEFAULT
        ):
            del arg_dict["buffsize"]
            del arg_dict["chunksize"]

        return arg_dict

    def _get_topics(self, only_checked=True):
        """
        Get the topics in the tree view

        :param only_checked: If true, return only those topics in the list which are checked
        :return: List of topics
        """
        topics = []
        for ind in range(0, self._model.rowCount()):
            # The topic column contains the topic name
            topic_ind = self._model.index(ind, 2)
            checkbox_ind = self._model.index(ind, 0)
            # Also check the checkbox status in the record column so we know whether or not to record
            # Must use CheckStateRole to get the state of the checkbox
            checked = (
                True
                if self._model.data(checkbox_ind, QtCore.Qt.CheckStateRole)
                == QtCore.Qt.Checked
                else False
            )
            if only_checked:
                if checked:
                    topics.append(self._model.data(topic_ind))
            else:
                topics.append(self._model.data(topic_ind))

        return topics

    def _generate_rosbag_command(self):
        """
        Convert the values of all the widgets into a rosbag record command list that can be run by subprocess.Popen

        :return:
        """
        value_dict = self._generate_arg_dict()

        cmd_arr = ["rosbag", "record"]
        for arg, val in value_dict.items():
            if val:
                # Must prepend -- to the args to make them into the args expected by the command
                if type(val) == bool:
                    cmd_arr.append("--" + arg)
                else:
                    if arg in ["output-name", "output-prefix"]:
                        # When generating the rosbag command, must make sure that directory expansions and so on are
                        # done, as popen will not do this.
                        val = os.path.abspath(os.path.expanduser(str(val)))
                    cmd_arr.extend(["--" + arg, str(val)])

        if "--output-name" in cmd_arr and "--output-prefix" in cmd_arr:
            bad_output = QMessageBox()
            bad_output.setWindowTitle("Invalid arguments")
            bad_output.setText(
                "You can't have both output file and file prefix set. Use one or the other to define the output "
                "filename. "
            )
            bad_output.exec()

            return []

        cmd_arr.extend(self._get_topics())

        return cmd_arr

    @staticmethod
    def _set_widget_from_dict(arg_dict, arg_name, widget, default_value):
        """
        Set the value of a widget using a dict containing the args
        :param arg_dict: Dict containing the args
        :param arg_name: Name of the argument to load to this widget
        :param widget: Widget to set the value of
        :param default_value: Default value to set if arg is empty
        :return:
        """
        arg_val = arg_dict.get(arg_name) or default_value
        if type(widget) == QSpinBox:
            widget.setValue(int(arg_val))
        if type(widget) == QCheckBox:
            widget.setCheckState(QtCore.Qt.Checked if arg_val else QtCore.Qt.Unchecked)
        if type(widget) == QLineEdit:
            widget.setText(arg_val)

    def _load_args(self, args):

        self._set_widget_from_dict(
            args, "buffsize", self._buffer_spin, RqtBagHelper.BUFFSIZE_DEFAULT
        )
        self._set_widget_from_dict(
            args, "chunksize", self._chunk_spin, RqtBagHelper.CHUNKSIZE_DEFAULT
        )
        buffsize = args.get("buffsize")
        chunksize = args.get("chunksize")

        if buffsize or chunksize:
            self._set_advanced(True)
        else:
            self._set_advanced(False)

        lz4 = args.get("lz4")
        bz2 = args.get("bz2")

        self._set_widget_from_dict(args, "lz4", self._lz4_check, False)
        self._set_widget_from_dict(args, "bz2", self._bz2_check, False)
        # If both compression types are set, default to using bz2
        rospy.logwarn("Both bz2 and lz4 compression are set to true, using bz2")
        if lz4 and bz2:
            self._lz4_check.setCheckState(QtCore.Qt.Unchecked)

        self._set_widget_from_dict(args, "duration", self._duration_spin, 0)
        self._set_widget_from_dict(args, "exclude", self._exclude_regex_edit, "")
        self._set_widget_from_dict(args, "limit", self._limit_spin, 0)
        self._set_widget_from_dict(args, "max-splits", self._max_split_spin, 0)
        self._set_widget_from_dict(args, "output-name", self._output_edit, "")
        self._set_widget_from_dict(args, "output-prefix", self._prefix_edit, "")
        self._set_widget_from_dict(args, "publish", self._publish_check, False)
        self._set_widget_from_dict(args, "regex", self._regex_edit, "")
        self._set_widget_from_dict(
            args, "repeat-latched", self._repeat_latched_check, False
        )
        self._set_widget_from_dict(args, "size", self._size_spin, 0)
        self._set_widget_from_dict(args, "split", self._split_check, False)
        self._set_widget_from_dict(args, "tcpnodelay", self._tcp_nodelay_check, False)
        self._set_widget_from_dict(args, "udp", self._udp_check, False)

    def _load_file(self):
        """
        Ask the user for a yaml file to load the bag definition
        :return:
        """
        options = QFileDialog.Options()
        filename, _ = QFileDialog.getOpenFileName(
            self._widget,
            "Open bag yaml definition",
            "",
            "Yaml files (*.yaml *.yml)",
            options=options,
        )
        if filename:
            with open(filename, "r") as f:
                bag_yaml = yaml.safe_load(f)
            if "topics" not in bag_yaml:
                rospy.logerr(
                    "Topics key does not exist in loaded yaml file. Will not do anything."
                )
                return

            for topic in bag_yaml["topics"]:
                self._add_topic(topic)

            self._load_args(bag_yaml["args"])

    def _save_file(self):
        """
        Open a dialog to ask the user where they want to save the yaml definition for this bag
        :return:
        """
        options = QFileDialog.Options()
        filename, _ = QFileDialog.getSaveFileName(
            self._widget,
            "Save bag yaml definition",
            "",
            "Yaml files (*.yaml *.yml)",
            options=options,
        )

        bag_def = {"args": self._generate_arg_dict(), "topics": self._get_topics()}

        with open(filename, "w") as f:
            f.write(yaml.safe_dump(bag_def))

    def _toggle_record(self):
        """
        Start or stop recording.
        :return:
        """
        if self._recording:
            # Because the rosbag record process actually spawns its own subprocess, we have to kill the whole process
            # group
            os.killpg(os.getpgid(self._process.pid), signal.SIGINT)
            self._record_button.setText("Record")
            self._recording = False
            rospy.loginfo("Stopped recording")
        else:
            cmd = self._generate_rosbag_command()
            if not cmd:
                return
            # Must run os.setsid to start rosbag record in a separate process group so we don't kill ourselves when
            # killing the children
            self._process = subprocess.Popen(cmd, preexec_fn=os.setsid)
            self._recording = True
            self._record_button.setText("Stop recording")
            rospy.loginfo("Started recording")

    def _compression_changed(self, widget, state):
        """
        Maintain valid state of compression checkboxes. Linked to the stateChanged signal

        :param widget: The checkbox whose state changed
        :param state: The new state of the checkbox
        :return:
        """
        if state == QtCore.Qt.Checked:
            if widget == self._lz4_check:
                self._bz2_check.setCheckState(QtCore.Qt.Unchecked)
            elif widget == self._bz2_check:
                self._lz4_check.setCheckState(QtCore.Qt.Unchecked)

    def _advanced_changed(self, state):
        """
        Enable or disable the advanced options depending on the state of the advanced checkbox

        :param state: New state of the advanced checkbox
        :return:
        """
        self._buffer_spin.setEnabled(state == QtCore.Qt.Checked)
        self._chunk_spin.setEnabled(state == QtCore.Qt.Checked)
