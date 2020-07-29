#!/usr/bin/env python
import rospy
from std_srvs.srv import SetBool, SetBoolResponse

def example_service_handler(data):
    print data
    response = SetBoolResponse()
    response.success = True
    response.message = 'Service called successfully!'
    return response

def hector_services():
    # service handler
    s = rospy.Service('/example/service', SetBool, example_service_handler)
    # set node
    rospy.init_node('example_services', anonymous=True)
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    try:
        hector_services()
    except rospy.ROSInterruptException:
        pass