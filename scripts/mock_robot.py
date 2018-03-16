#! /usr/bin/env python

import rospy
import actionlib
import refills_msgs.msg


class MockRobot(object):
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, refills_msgs.msg.ScanningAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        if goal.type is refills_msgs.msg.ScanningGoal.UNKNOWN_TYPE:
            rospy.loginfo('Received invalid command. Aborting.')
            self._as.set_aborted(refills_msgs.msg.ScanningResult(
                error_code=refills_msgs.msg.ScanningResult.UNKNOWN_ERROR,
                error_string = "Received scanning command of type 'UNKNOWN_TYPE'."))
        elif goal.type is refills_msgs.msg.ScanningGoal.COMPLETE_SCAN:
            rospy.loginfo('Start scanning %s shelves', len(goal.loc_id))
            feedback = refills_msgs.msg.ScanningFeedback()

            for index, loc_id in enumerate(goal.loc_id):
                feedback.progress = 100 * index / len(goal.loc_id)
                feedback.current_loc_id = loc_id

                if self._as.is_preempt_requested():
                    rospy.loginfo('Got preempt request.')
                    self._as.set_preempted()
                    return

                self._as.publish_feedback(feedback)
                rospy.sleep(rospy.Duration(1))

            feedback.progress = 100.0
            feedback.current_loc_id = ''
            self._as.publish_feedback(feedback)

            self._as.set_succeeded(refills_msgs.msg.ScanningResult())


if __name__ == '__main__':
    rospy.init_node('mock_robot')
    server = MockRobot('scanning_action')
    rospy.spin()