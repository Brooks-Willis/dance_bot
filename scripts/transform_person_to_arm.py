#!/usr/bin/env python
import rospy
from dance_bot.msg import PathWithTime, ArmPos, Path, ARMarker

def transform_coords(person_coord, p_rad):
    p_x, p_y, p_z = person_coord
    robot_coord = [coord * 7500.0 / p_rad for coord in [p_y, p_z, p_x]]
    return robot_coord

def transform_path(person_path, p_rad):
    return [transform_coords(coord, p_rad) for coord in person_path]


class PathCompiler(object):

    def __init__(self, joint_num, arm_radius, path_length=100):
        rospy.init_node('compile_path')
        self.joint = joint_num
        self.path_length = max(2,path_length)
        self.arm_radius = arm_radius
        self.poses = []

        self.sub = rospy.Subscriber('RAAAGGHEHOIGWEOIHWGEWIHEGOIH',ARMarker,self.got_marker)
        self.pub = rospy.Publisher('training_path_'+str(joint_num),PathWithTime)

    def got_marker(self, data):
        if data.id == self.joint:
            x = data.pose.pose.position.x
            y = data.pose.pose.postition.y
            z = data.pose.pose.position.z
            time = data.header.stamp
            self.poses.append(([x,y,x],time))
        if len(self.poses) >= self.path_length:
            pos, times = [list(x) for x in zip(*self.poses)]
            self.poses = []
            self.publish(pos,times)

    def publish(self,pos,times):
        robot_pos = transform_path(pos, self.arm_radius)
        path = Path(path=[ArmPos(x=p[0],y=p[1],z=p[2]) for p in robot_pos])
        path_with_time = PathWithTime(path=path, times=times)
        self.pub.publish(path_with_time)




if __name__ == "__main__":
    rospy.init_node('compile_path')

    p_rad = 1.0

    p_coords = [[0,0,0],
                [1,0,0],
                [0,1,0],
                [0,0,1],
                [0,-1,0],
                [.5,.5,.5]]

    for coord in p_coords:
        print "Person: ", coord
        print "Robot Arm: ", transform_coords(coord,p_rad)
        print "\n"

    print "whole path:", transform_path(p_coords,p_rad)

    path_comp = PathCompiler(1,1)
    rospy.spin()