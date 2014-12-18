#!/usr/bin/env python
import rospy 
from dance_bot.msg import PlanRequest
import time
import sys


if __name__=="__main__":
    rospy.init_node('requesting_funky_move')
    request_publisher = rospy.Publisher("plan_request", PlanRequest)

    while not(rospy.is_shutdown()):
    
        dmp_name = raw_input('Which dmp do you want to learn? (eg test1)\n')
        start_pos = input('Where do you want to start? (eg [0,0,7500])\n')
        goal = input('Where do you want to end? (eg [400,-800,5000])\n')

        request = PlanRequest(dmp_name=dmp_name,
                              start_pos=start_pos,
                              start_vel=[0,0,0],
                              goal=goal)

        print "Sending request:", request
        request_publisher.publish(request)

