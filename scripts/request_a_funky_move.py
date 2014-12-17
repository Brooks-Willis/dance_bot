#!/usr/bin/env python
import rospy 
from dance_bot.msg import PlanRequest
import time


if __name__=="__main__":
    rospy.init_node('requesting_funky_move')
    request_publisher = rospy.Publisher("plan_request", PlanRequest)
    print request_publisher

    request = PlanRequest(dmp_name='test4',
                          start_pos=[300,1000,5500],
                          start_vel=[0,0,0],
                          goal=[-1000,1000,5500])

    # request = PlanRequest(dmp_name='test1_500_1',
    #                       start_pos=[-1500,0,-2000],
    #                       start_vel=[0,0,0],
    #                       goal=[-1300,-400,-2500])
    print request
    time.sleep(1)
    request_publisher.publish(request)
    rospy.spin()

