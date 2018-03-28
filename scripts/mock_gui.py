#!/usr/bin/env python

from PyQt4 import QtGui
from PyQt4.QtCore import QThread, pyqtSignal, Qt
from json_prolog import json_prolog

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

        self.execButton.clicked.connect(self.start)
        self.cancelButton.clicked.connect(self.cancel)
        self.syncButton.clicked.connect(self.sync_with_knowrob)
        self.progressBar.setValue(0)

        self.client = None
        self.action_thread = None
        self.prolog = None
        self.ros_setup()

    def ros_setup(self):
        self.client = actionlib.SimpleActionClient('/scanning_action', refills_msgs.msg.ScanningAction)
        if not self.client.wait_for_server(rospy.Duration(0.5)):
            rospy.logwarn("Waiting for action server at '/scanning_action' to appear. Please make sure it is up.")
        self.client.wait_for_server()

        self.prolog = json_prolog.Prolog()
        try:
            self.prolog.wait_for_service(0.5)
        except rospy.ROSException:
            rospy.logwarn("Waiting for json_prolog server at default namespace to appear. Please make sure it is up.")
        self.prolog.wait_for_service()

    def start(self):
        self.action_thread = ActionThread(self.client, self.read_target_locs())
        self.action_thread.feedback.connect(self.feedback)
        self.action_thread.finished.connect(self.done)
        self.action_thread.result.connect(self.result)
        self.action_thread.start()
        self.cancelButton.setEnabled(True)
        self.execButton.setEnabled(False)

    def sync_with_knowrob(self):
        self.treeWidget.clear()
        query_string = "findall(A, owl_individual_of(A, dmshop:'DMShelfSystem'), ShelfSystems)."
        systems = self.prolog.once(query_string)['ShelfSystems']
        for system in systems:
            parent = QtGui.QTreeWidgetItem(self.treeWidget)
            parent.setText(0, system)
            parent.setFlags(parent.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable)
            query_string = "findall(A, owl_has('{}', knowrob:'properPhysicalParts', A), ShelfMeters).".format(system)
            meters = self.prolog.once(query_string)['ShelfMeters']
            for meter in meters:
                child = QtGui.QTreeWidgetItem(parent)
                child.setFlags(child.flags() | Qt.ItemIsUserCheckable)
                child.setText(0, meter)
                child.setCheckState(0, Qt.Unchecked)
        if self.treeWidget.topLevelItemCount() > 0:
            self.execButton.setEnabled(True)
        else:
            self.execButton.setEnabled(False)

    def read_target_locs(self):
        target_locs = []
        iterator = QtGui.QTreeWidgetItemIterator(self.treeWidget, QtGui.QTreeWidgetItemIterator.Checked)
        while iterator.value():
            item = iterator.value()
            # TODO: smarter filtering of unwanted loc ids
            if not 'ShelfSystem' in item.text(0):
                target_locs.append(str(item.text(0)))
            iterator += 1
        return target_locs

    def feedback(self, msg):
        self.statusbar.showMessage(msg.current_loc_id)
        self.progressBar.setValue(msg.progress)

    def done(self):
        self.cancelButton.setEnabled(False)
        self.execButton.setEnabled(True)

    def cancel(self):
        self.action_thread.cancel_request.emit()

    def result(self, msg):
        if msg is actionlib.GoalStatus.SUCCEEDED:
           self.statusbar.showMessage("Success.")
           self.progressBar.setValue(100)
        elif msg is actionlib.GoalStatus.ABORTED:
            self.statusbar.showMessage("Aborted: {}".format(msg.error_string))
        elif msg is actionlib.GoalStatus.PREEMPTED:
            self.statusbar.showMessage("Canceled.")


if __name__ == "__main__":
    rospy.init_node("mock_sms")
    app = QtGui.QApplication(sys.argv)
    ui = MockGui()
    ui.show()
    sys.exit(app.exec_())

