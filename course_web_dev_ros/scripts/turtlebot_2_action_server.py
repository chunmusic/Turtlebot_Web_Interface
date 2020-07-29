#! /usr/bin/env python
import rospy
import time
import actionlib

from course_web_dev_ros.msg import WaypointActionFeedback, WaypointActionResult, WaypointActionAction
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import math

class WaypointActionClass(object):

    # create messages that are used to publish feedback/result
    _feedback = WaypointActionFeedback()
    _result = WaypointActionResult()

    # topics
    _pub_cmd_vel = None
    _sub_odom = None

    # go to point vars
    # robot state variables
    _position = Point()
    _yaw = 0
    # machine state
    _state = 'idle'
    # goal
    _des_pos = Point()
    # parameters
    _yaw_precision = math.pi / 90 # +/- 2 degree allowed
    _dist_precision = 0.3

    def __init__(self):
        # creates the action server
        self._as = actionlib.SimpleActionServer("turtlebot2_action_service_as", WaypointActionAction, self.goal_callback, False)
        self._as.start()

        # define a loop rate
        self._rate = rospy.Rate(10)

        # topics
        self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._sub_odom = rospy.Subscriber('/odom', Odometry, self._clbk_odom)
        print 'Action server started'

    def _clbk_odom(self, msg):
        # position
        self._position = msg.pose.pose.position

        # yaw
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self._yaw = euler[2]

    def goal_callback(self, goal):
        print 'goal %s received' % str(goal)

        # helper variables
        success = True

        # define desired position and errors
        self._des_pos = goal.position
        desired_yaw = math.atan2(self._des_pos.y - self._position.y, self._des_pos.x - self._position.x)
        err_pos = math.sqrt(pow(self._des_pos.y - self._position.y, 2) + pow(self._des_pos.x - self._position.x, 2))
        err_yaw = desired_yaw - self._yaw

        # perform task
        while err_pos > self._dist_precision and success:
            # update vars
            desired_yaw = math.atan2(self._des_pos.y - self._position.y, self._des_pos.x - self._position.x)
            err_yaw = desired_yaw - self._yaw
            err_pos = math.sqrt(pow(self._des_pos.y - self._position.y, 2) + pow(self._des_pos.x - self._position.x, 2))
            print self._yaw
            print desired_yaw
            print err_yaw
            # logic goes here
            if self._as.is_preempt_requested():
                # cancelled
                print 'The goal has been cancelled/preempted'
                self._as.set_preempted()
                success = False
            elif math.fabs(err_yaw) > self._yaw_precision:
                # fix yaw
                print 'fix yaw'
                self._state = 'fix yaw'
                twist_msg = Twist()
                twist_msg.angular.z = 0.5 if err_yaw > 0 else -0.5
                self._pub_cmd_vel.publish(twist_msg)
            else:
                # go to point
                print 'go to point'
                self._state = 'go to point'
                twist_msg = Twist()
                twist_msg.linear.x = 0.6
                twist_msg.angular.z = 0
                # twist_msg.angular.z = 0.1 if err_yaw > 0 else -0.1
                self._pub_cmd_vel.publish(twist_msg)

            # send feedback
            self._feedback.position = self._position
            self._feedback.state = self._state
            self._as.publish_feedback(self._feedback)

            # loop rate
            self._rate.sleep()

        # stop
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        self._pub_cmd_vel.publish(twist_msg)

        # return success
        if success:
            self._result.success = True
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('turtlebot2_action_service_as')
    WaypointActionClass()
    rospy.spin()