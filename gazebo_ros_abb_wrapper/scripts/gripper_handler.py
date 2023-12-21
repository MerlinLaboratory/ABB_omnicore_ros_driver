#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Trigger, TriggerResponse

class GripperHandler:

    def __init__(self):
        self.gripper_pub = rospy.Publisher('/gripper_controller/command',JointTrajectory, queue_size=1)
        self.open_srv = rospy.Service('/open_gripper', Trigger, self.openGrippperReq)
        self.close_srv = rospy.Service('/close_gripper', Trigger, self.closeGrippperReq)
            
    def openGripper(self):
        self.moveGripper(0.025)
        
    def closeGripper(self):
        self.moveGripper(0.0)

    def moveGripper(self, n):
        traj = JointTrajectory()
        names = ["gripper_joint_l", "gripper_joint_r"]
        traj.joint_names = names
        goal = JointTrajectoryPoint()
        goal.positions = [n]*2
        goal.time_from_start = rospy.Duration(1)
        traj.points.append(goal)
        self.gripper_pub.publish(traj)

    def openGrippperReq(self, req):
        self.openGripper()
        rospy.sleep(0.5)
        res = TriggerResponse()
        res.success = True
        return res

    def closeGrippperReq(self, req):
        self.closeGripper()
        rospy.sleep(0.5)
        res = TriggerResponse()
        res.success = True
        return res    

def main(args=None):
    rospy.init_node('gripper_handler', anonymous=True)

    gripper_handler = GripperHandler()
    rospy.sleep(1)
    gripper_handler.openGripper()

    rospy.spin()

if __name__ == '__main__':
    main()