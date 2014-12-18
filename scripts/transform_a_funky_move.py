#!/usr/bin/env python
import rospy
from dance_bot.msg import ARMarkers
from std_msgs.msg import String
import numpy as np
import csv
import pandas
import sys

def transform_coords(person_coord, p_rad):
    p_x, p_y, p_z = person_coord
    robot_coord = [coord * 7500.0 / p_rad for coord in [p_y, p_z, p_x]]
    return robot_coord

def transform_path(person_path, p_rad):
    return [transform_coords(coord, p_rad) for coord in person_path]

class PathCompiler(object):

    def __init__(self, output_path, arm_radius, hand_num=0, elbow_num=1, shoulder_num=2):
        rospy.init_node('compile_path')
        self.nums = [hand_num, elbow_num, shoulder_num]
        self.arm_radius = arm_radius
        self.time_offset = False

        self.sub = rospy.Subscriber('ar_pose_marker',ARMarkers,self.got_markers)
        self.sub = rospy.Subscriber('kill_sig',String,self.kill_sig)

        self.outfile = open(output_path,'w')
        self.writer = csv.writer(self.outfile, delimiter=',')

        headers = ['times']
        for name in ['hand','elbow','shoulder']:
            headers = headers + [name+suf for suf in ['_x','_y','_z']]
        
        self.writer.writerow(headers)

        self.shutdown = False

    def got_markers(self, data):
        if self.shutdown:
            return
        found_poses = {}
        for marker in data.markers:
            if not self.time_offset:
                self.time_offset = marker.header.stamp.to_sec()
            x = marker.pose.pose.position.x
            y = marker.pose.pose.position.y
            z = marker.pose.pose.position.z
            time = marker.header.stamp.to_sec() - self.time_offset
            if marker.id in self.nums:
                found_poses[marker.id] = [x,y,z]

        if len(found_poses.keys()) == len(self.nums):
            row = [time]
            shoulder = found_poses[self.nums[-1]]
            for i in self.nums:
                joint = found_poses[i]
                arm_pose = [joint[i] - shoulder[i] for i in [0,1,2]]
                robot_pose = transform_coords(arm_pose, self.arm_radius)
                row = row + robot_pose
            self.writer.writerow(row)

    def kill_sig(self,data):
        self.outfile.close()
        self.shutdown = True

    def execute(self):
        r = rospy.Rate(1)
        while not self.shutdown:
            r.sleep()


if __name__ == "__main__":

    if len(sys.argv) >= 2 and sys.argv[1][:2] != "__":
        folder = sys.argv[1]
    else:
        print "first argument should be the folder path to save in (with a slash on the end)"
        sys.exit(0)

    rospy.init_node('compile_path')

    r = rospy.Rate(.2)
    r.sleep()
    while not(rospy.is_shutdown()):

        filename = raw_input("Enter the name of the file you want to save (eg test1) and press \
enter to begin recording. Either send a string (std_msgs/String) to the topic \
kill_sig or use Ctrl-C to stop recording\n")

        compiler = PathCompiler(folder+filename+'.csv', 1)
        compiler.execute()

    rospy.spin()