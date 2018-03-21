#!/usr/bin/env python

from PyQt4 import QtGui
from PyQt4.QtCore import QThread, pyqtSignal
from refills_mock_gui.layout import Ui_MainWindow
import sys
import rospy
import actionlib
import refills_msgs.msg


class ActionThread(QThread):

    feedback = pyqtSignal('PyQt_PyObject', name='feedback')
    cancel_request = pyqtSignal(name="preempt_request")
    result = pyqtSignal('PyQt_PyObject', name='result')

    def __init__(self, client, target_locs):
        QThread.__init__(self)
        self.client = client
        self.target_locs = target_locs
        self.cancel_request.connect(self.cancel)

    def __del__(self):
        self.wait()

    def run(self):
        goal = refills_msgs.msg.ScanningGoal(type=refills_msgs.msg.ScanningGoal.COMPLETE_SCAN, loc_id=self.target_locs)
        self.client.send_goal(goal, feedback_cb=self.feedback_cb)
        self.client.wait_for_result()
        self.result.emit(self.client.get_state())

    def feedback_cb(self, msg):
        self.feedback.emit(msg)

    def cancel(self):
        self.client.cancel_all_goals()


class MockGui(QtGui.QMainWindow, Ui_MainWindow):
    def __init__(self):
        super(self.__class__, self).__init__()
        self.setupUi(self)

        self.listWidget.setSelectionMode(QtGui.QAbstractItemView.ExtendedSelection)
        self.execButton.clicked.connect(self.start)
        self.cancelButton.clicked.connect(self.cancel)
        self.cancelButton.setEnabled(False)
        self.progressBar.setValue(0)

    def ros_setup(self):
        self.client = actionlib.SimpleActionClient('/scanning_action', refills_msgs.msg.ScanningAction)
        if not self.client.wait_for_server(rospy.Duration(1)):
            raise RuntimeError("Could not connect to action server of '/scanning_action'.")

        for loc_id in rospy.get_param("/mock_gui/loc_ids"):
            self.listWidget.addItem(QtGui.QListWidgetItem(loc_id))

    def start(self):
        target_locs = [str(item.text()) for item in self.listWidget.selectedItems()]
        self.action_thread = ActionThread(self.client, target_locs)
        self.action_thread.feedback.connect(self.feedback)
        self.action_thread.finished.connect(self.done)
        self.action_thread.result.connect(self.result)
        self.action_thread.start()
        self.cancelButton.setEnabled(True)
        self.execButton.setEnabled(False)

    def feedback(self, msg):
        self.statusbar.showMessage(msg.current_loc_id)
        self.progressBar.setValue(msg.progress)

    def done(self):
        self.cancelButton.setEnabled(False)
        self.execButton.setEnabled(True)

    def cancel(self):
        self.statusbar.showMessage("Canceled.")
        self.action_thread.cancel_request.emit()

    def result(self, msg):
        if msg is actionlib.GoalStatus.SUCCEEDED:
           self.statusbar.showMessage("Success.")
           self.progressBar.setValue(100)


if __name__ == "__main__":
    rospy.init_node("mock_sms")
    app = QtGui.QApplication(sys.argv)
    ui = MockGui()
    ui.ros_setup()
    ui.show()
    sys.exit(app.exec_())

