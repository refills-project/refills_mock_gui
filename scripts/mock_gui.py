#!/usr/bin/env python

from PyQt4 import QtCore, QtGui
from refills_mock_gui.layout import Ui_MainWindow
import sys
import rospy

class MockSmsMainWindow(Ui_MainWindow):
    def __init__(self):
        super(MockSmsMainWindow, self).__init__()

if __name__ == "__main__":
    rospy.init_node("mock_sms")
    app = QtGui.QApplication(sys.argv)
    MainWindow = QtGui.QMainWindow()
    ui = MockSmsMainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

