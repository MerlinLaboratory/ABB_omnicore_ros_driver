#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def Publisher():
    eff_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=1)
    vel_pub = rospy.Publisher('/vel_arm_controller/command', JointTrajectory, queue_size=1)
    rospy.init_node('init_traj_publisher', anonymous=True)
    traj = JointTrajectory()
    names = []
    for i in range(7):
        names.append("joint_"+str(i+1))
    traj.joint_names = names
    goal = JointTrajectoryPoint()
    goal.positions = [0]*7
    goal.time_from_start = rospy.Duration(1)
    traj.points.append(goal)
    rospy.sleep(1)
    eff_pub.publish(traj)
    vel_pub.publish(traj)

if __name__ == '__main__':
    try:
        Publisher()
    except rospy.ROSInterruptException:
        pass