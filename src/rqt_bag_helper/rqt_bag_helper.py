import os
import rospy
import rospkg
import yaml

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

        # Set up the model used for sorting and filtering the fields
        self._sort_model = QSortFilterProxyModel()
        self._sort_model.setSourceModel(self._model)

        self._topic_tree = self._widget.findChildren(QTreeView, "topicTree")[0]
        self._topic_tree.setModel(self._sort_model)

        self._model.setHorizontalHeaderLabels(["Record", "Rate", "Topic"])
        # There are only two sections so they can't be moved anyway
        self._topic_tree.header().setSectionsMovable(False)
        # self._topic_tree.header().setDefaultSectionSize(200)
        self._topic_tree.setSortingEnabled(True)

        self._widget.findChildren(QPushButton, "loadButton")[0].clicked.connect(
            self._load_file
        )

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
        self._bz2_check = self._widget.findChildren(QCheckBox, "bz2CheckBox")[0]
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
        check_item.setEnabled(False)
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

    def _set_widget_from_dict(self, arg_dict, arg_name, widget, default_value):
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

        compression_type = args.get("compression").lower()
        if compression_type and compression_type in ["lz4", "bz2"]:
            if compression_type == "lz4":
                self._lz4_check.setCheckState(QtCore.Qt.Checked)
            elif compression_type == "bz2":
                self._bz2_check.setCheckState(QtCore.Qt.Checked)
        else:
            self._lz4_check.setCheckState(QtCore.Qt.Unchecked)
            self._bz2_check.setCheckState(QtCore.Qt.Unchecked)

        self._set_widget_from_dict(args, "duration", self._duration_spin, 0)
        self._set_widget_from_dict(args, "exclude_regex", self._exclude_regex_edit, "")
        self._set_widget_from_dict(args, "limit", self._limit_spin, 0)
        self._set_widget_from_dict(args, "max_splits", self._max_split_spin, 0)
        self._set_widget_from_dict(args, "name", self._output_edit, "")
        self._set_widget_from_dict(args, "prefix", self._prefix_edit, "")
        self._set_widget_from_dict(args, "publish", self._publish_check, False)
        self._set_widget_from_dict(args, "regex", self._regex_edit, "")
        self._set_widget_from_dict(
            args, "repeat_latched", self._repeat_latched_check, False
        )
        self._set_widget_from_dict(args, "size", self._size_spin, 0)
        self._set_widget_from_dict(args, "split", self._split_check, False)
        self._set_widget_from_dict(args, "tcpnodelay", self._tcp_nodelay_check, False)
        self._set_widget_from_dict(args, "udp", self._udp_check, False)

    def _load_file(self):
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
