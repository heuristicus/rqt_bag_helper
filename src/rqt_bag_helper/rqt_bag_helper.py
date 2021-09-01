import os
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import (
    QWidget,
    QTreeView,
    QLineEdit,
    QPushButton,
    QLabel,
)
from python_qt_binding.QtGui import QStandardItemModel, QStandardItem
from python_qt_binding.QtCore import QSortFilterProxyModel
import python_qt_binding.QtCore as QtCore


class RqtBagHelper(Plugin):

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
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (" (%d)" % context.serial_number())
            )

        self._model = QStandardItemModel()

        # Set up the model used for sorting and filtering the fields
        self._sort_model = QSortFilterProxyModel()
        self._sort_model.setSourceModel(self._model)

        self._param_tree = self._widget.findChildren(QTreeView, "topicTree")[0]
        self._param_tree.setModel(self._sort_model)

        self._model.setHorizontalHeaderLabels(["Record", "Topic", "Rate"])
        # There are only two sections so they can't be moved anyway
        self._param_tree.header().setSectionsMovable(False)
        self._param_tree.header().setDefaultSectionSize(200)
        self._param_tree.setSortingEnabled(True)

        context.add_widget(self._widget)
