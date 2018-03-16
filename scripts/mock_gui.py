#!/usr/bin/env python

from PyQt4 import QtGui
from refills_mock_gui.layout import Ui_MainWindow
import sys
import rospy
import actionlib
import refills_msgs.msg


class MockSmsMainWindow(Ui_MainWindow):
    def __init__(self):
        super(MockSmsMainWindow, self).__init__()
        self.target_locs = []
        self.client = None

    def ros_setup(self):
        self.client = actionlib.SimpleActionClient('/scanning_action', refills_msgs.msg.ScanningAction)
        if not self.client.wait_for_server(rospy.Duration(1)):
            raise RuntimeError("Could not connect to action server of '/scanning_action'.")

        for loc_id in rospy.get_param("/mock_gui/loc_ids"):
            self.listWidget.addItem(QtGui.QListWidgetItem(loc_id))
        self.listWidget.setSelectionMode(QtGui.QAbstractItemView.ExtendedSelection)

        self.execButton.clicked.connect(self.start_scan)
        self.cancelButton.clicked.connect(self.cancel_scan)

        self.progressBar.setValue(0)

    def start_scan(self):
        self.target_locs = [str(item.text()) for item in self.listWidget.selectedItems()]
        goal = refills_msgs.msg.ScanningGoal(type=refills_msgs.msg.ScanningGoal.COMPLETE_SCAN, loc_id=self.target_locs)
        self.client.send_goal(goal, feedback_cb=self.feedback_cb)
        self.client.wait_for_result()
        if self.client.get_state() is actionlib.GoalStatus.SUCCEEDED:
            self.statusbar.showMessage("Done with scanning.")
            self.progressBar.setValue(100)

    def cancel_scan(self):
        self.client.cancel_all_goals()
        self.statusbar.showMessage("Canceled scanning.")

    def feedback_cb(self, msg):
        self.statusbar.showMessage(msg.current_loc_id)
        self.progressBar.setValue(msg.progress)


if __name__ == "__main__":
    rospy.init_node("mock_sms")
    app = QtGui.QApplication(sys.argv)
    MainWindow = QtGui.QMainWindow()
    ui = MockSmsMainWindow()
    ui.setupUi(MainWindow)
    ui.ros_setup()
    MainWindow.show()
    sys.exit(app.exec_())

