#!/usr/bin/env python

from PyQt4 import QtCore, QtGui
from refills_mock_gui.layout import Ui_MainWindow
import sys
import rospy
import actionlib
import refills_msgs.msg

class MockSmsMainWindow(Ui_MainWindow):
    def __init__(self):
        super(MockSmsMainWindow, self).__init__()

    def ros_setup(self):
        self.client = actionlib.SimpleActionClient('/scanning_action', refills_msgs.msg.ScanningAction)
        if not self.client.wait_for_server(rospy.Duration(1)):
            raise RuntimeError("Could not connect to action server of '/scanning_action'.")

        for loc_id in rospy.get_param("/mock_gui/loc_ids"):
            self.listWidget.addItem(QtGui.QListWidgetItem(loc_id))
        self.listWidget.itemClicked.connect(self.set_target_loc)

        self.execButton.clicked.connect(self.start_scan)
        self.cancelButton.clicked.connect(self.cancel_scan)

    def set_target_loc(self, item):
        self.target_loc = str(item.text())

    def start_scan(self):
        goal = refills_msgs.msg.ScanningGoal(type=refills_msgs.msg.ScanningGoal.COMPLETE_SCAN, loc_id=[self.target_loc])
        self.client.send_goal(goal, feedback_cb=self.feedback_cb)
        self.client.wait_for_result()

    def cancel_scan(self):
        # TODO: complete me

    def feedback_cb(self, msg):
        # TODO: complete me
        pass


if __name__ == "__main__":
    rospy.init_node("mock_sms")
    app = QtGui.QApplication(sys.argv)
    MainWindow = QtGui.QMainWindow()
    ui = MockSmsMainWindow()
    ui.setupUi(MainWindow)
    ui.ros_setup()
    MainWindow.show()
    sys.exit(app.exec_())

