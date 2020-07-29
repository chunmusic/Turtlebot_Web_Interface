#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolResponse

pub = None

def perform_task():
    # global vars
    global pub

    # read parameters
    linear_x = float(rospy.get_param("/web_param/linear_x"))
    angular_z = float(rospy.get_param("/web_param/angular_z"))

    msg = Twist()
    msg.linear.x = linear_x
    msg.angular.z = angular_z

    pub.publish(msg)

def robot_services():
    # global vars
    global pub
    # publisher
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    # set node
    rospy.init_node('robot_services', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    linear_x = rospy.set_param("/web_param/linear_x", 0)
    angular_z = rospy.set_param("/web_param/angular_z", 0)

    while not rospy.is_shutdown():
        perform_task()
        rate.sleep()

if __name__ == '__main__':
    try:
        robot_services()
    except rospy.ROSInterruptException:
        pass