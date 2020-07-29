#! /usr/bin/env python
import rospy
import time
import actionlib

from course_web_dev_ros.msg import ExampleActionFeedback, ExampleActionResult, ExampleActionAction
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class ExampleActionClass(object):

    # create messages that are used to publish feedback/result
    _feedback = ExampleActionFeedback()
    _result = ExampleActionResult()

    def __init__(self):
        # creates the action server
        self._as = actionlib.SimpleActionServer("example_action_service_as", ExampleActionAction, self.goal_callback, False)
        self._as.start()
        self.rate = rospy.Rate(1)
        print 'Action server started'

    def goal_callback(self, goal):
        print 'goal %s received' % str(goal)

        # helper variables
        success = True

        # define vars
        self._count = 0

        # perform task
        while self._count < goal.total and success:
            print self._count
            self.rate.sleep()
            if self._as.is_preempt_requested():
                # cancelled
                print 'The goal has been cancelled/preempted at %s' % str(self._count)
                self._as.set_preempted()
                success = False
            else:
                # keep going
                self._count = self._count + 1
                self._feedback.progress = self._count
                self._as.publish_feedback(self._feedback)

        # return success
        if success:
            print 'result is %s' % str(self._count)
            self._result.result = self._count
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('example_action_service_as')
    ExampleActionClass()
    rospy.spin()