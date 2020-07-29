#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolResponse
from sensor_msgs.msg import Range

# task status
# 1 - takeoff
# 2 - landing
status = 0
pub = None

# subscribed data
height = 0

def clbk_sonar(data):
    # get global vars
    global height

    # process data
    height = data.range

def service_takeoff(req):
    # get global vars
    global pub, height, status

    # define response
    response = SetBoolResponse()

    # busy or set task
    if status is not 0:
        print 'busy'
        response.success = False
        response.message = 'Drone is busy!'
    else:
        status = 1
        response.success = True
        response.message = 'Takeoff performed successfully!'

    # return response
    return response

def service_landing(req):
    # get global vars
    global pub, height, status

    # define response
    response = SetBoolResponse()

    # busy or set task
    if status is not 0:
        print 'busy'
        response.success = False
        response.message = 'Drone is busy!'
    else:
        status = 2
        response.success = True
        response.message = 'Takeoff performed successfully!'

    # return response
    return response

def perform_task():
    # get global vars
    global pub, height, status

    # do nothing
    if status is 0:
        pass

    # takeoff
    if status is 1:
        perform_takeoff()

    # landing
    if status is 2:
        perform_landing()

def perform_takeoff():
    # get global vars
    global pub, height, status

    # define vars
    rate = rospy.Rate(10)
    print 'perform takeoff'
    msg = Twist()

    # takeoff
    if height < 1.5:
        msg.linear.z = 0.2
        while height < 1.5:
            pub.publish(msg)
            rate.sleep()

    # stop
    print 'stop'
    msg.linear.z = 0
    pub.publish(msg)
    rate.sleep()
    status = 0

def perform_landing():
    # get global vars
    global pub, height, status

    # define vars
    rate = rospy.Rate(10)
    print 'perform landing'
    msg = Twist()

    # landing
    if height > 0.16:
        msg.linear.z = -0.2
        while height > 0.16:
            pub.publish(msg)
            rate.sleep()

    # stop
    print 'stop'
    msg.linear.z = 0
    pub.publish(msg)
    rate.sleep()
    status = 0

def hector_services():
    # global vars
    global pub
    # service handler
    s = rospy.Service('/hector_services/takeoff', SetBool, service_takeoff)
    s = rospy.Service('/hector_services/landing', SetBool, service_landing)
    # publisher
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    # subscriber
    sub = rospy.Subscriber('/sonar_height', Range, clbk_sonar)
    # set node
    rospy.init_node('hector_services', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        perform_task()
        rate.sleep()

if __name__ == '__main__':
    try:
        hector_services()
    except rospy.ROSInterruptException:
        pass