#!/usr/bin/env python

from .uvs_gui import UVSGuiWidget
from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import Qt, QTimer
import rospy


class UVSGuiPlugin(Plugin):
    def __init__(self, context):
        super(UVSGuiPlugin, self).__init__(context)
        if context.serial_number() > 1:
            raise RuntimeError("You may not run more than one instance of visual_servoing_gui.")
        self.setObjectName("UVS Plugin")
        self._widget = UVSGuiWidget()
        context.add_widget(self._widget)

        # self._update_parameter_timer = QTimer(self)
        # self._update_parameter_timer.timeout.connect(self._widget.pub_check)
        # self._update_parameter_timer.start(100)
