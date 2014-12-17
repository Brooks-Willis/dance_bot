#!/usr/bin/env python
import rospy 
from dance_bot.msg import PlanRequest
import time


if __name__=="__main__":
    rospy.init_node('requesting_funky_move')
    request_publisher = rospy.Publisher("plan_request", PlanRequest)
    print request_publisher

    request = PlanRequest(dmp_name='test1.json', start_pos=[0,0,7500], start_vel=[0,0,0], goal=[-1500,-500,-2500])
    print request
    time.sleep(1)
    request_publisher.publish(request)
    rospy.spin()

