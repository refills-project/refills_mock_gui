#!/usr/bin/env python

from PyQt4 import QtCore, QtGui
from refills_mock_gui.layout import Ui_MainWindow
import sys
import rospy

class MockSmsMainWindow(Ui_MainWindow):
    def __init__(self):
        super(MockSmsMainWindow, self).__init__()

    def ros_setup(self):
        
        for loc_id in rospy.get_param("/mock_gui/loc_ids"):
            self.listWidget.addItem(QtGui.QListWidgetItem(loc_id))
        self.listWidget.itemClicked.connect(self.set_target_loc)

    def set_target_loc(self, item):
        self.target_loc = str(item.text)


if __name__ == "__main__":
    rospy.init_node("mock_sms")
    app = QtGui.QApplication(sys.argv)
    MainWindow = QtGui.QMainWindow()
    ui = MockSmsMainWindow()
    ui.setupUi(MainWindow)
    ui.ros_setup()
    MainWindow.show()
    sys.exit(app.exec_())

