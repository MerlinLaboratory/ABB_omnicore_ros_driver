#!/usr/bin/env python

from std_srvs.srv import TriggerRequest,TriggerResponse,Trigger
from abb_wrapper_msgs.srv import move_gripper_to, move_gripper_toResponse

from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import rospy

class GripperSimul:

    def __init__(self):
        rospy.init_node('gripper_simulation')
        self.grip_in  = rospy.Service('/grip_in', Trigger, self.close_gripper)
        self.grip_out = rospy.Service('/grip_out', Trigger, self.open_gripper)
        self.grip_to = rospy.Service('/move_gripper_to', move_gripper_to, self.gripper_to)
        
        # self.pub_command       = rospy.Publisher("/gripper_controller/command", Float64MultiArray, queue_size=1,latch=True)
        self.pub_command       = rospy.Publisher("/gripper_controller/command", JointTrajectory, queue_size=1,latch=True)


    def open_gripper(self,req):
        # open_msg = Float64MultiArray()
        # open_msg.data = [2.5/100,2.5/100]
        # open_msg.layout.data_offset = 0
        open_msg = JointTrajectory()
        open_msg.joint_names = ["gripper_joint_l", "gripper_joint_r"]
        point = JointTrajectoryPoint()
        point.positions = [2.5/100,2.5/100]
        point.time_from_start = rospy.Time(1)
        open_msg.points.append(point)

        self.pub_command.publish(open_msg)
        resp = TriggerResponse()
        resp.success = True
        resp.message = "Gripper Opened"
        return resp

    def gripper_to(self, req):
        # open_msg = Float64MultiArray()
        # open_msg.data = [pos,pos]
        # open_msg.layout.data_offset = 0
        pos = req.position
        open_msg = JointTrajectory()
        open_msg.joint_names = ["gripper_joint_l", "gripper_joint_r"]
        point = JointTrajectoryPoint()
        point.positions = [pos,pos]
        point.time_from_start = rospy.Time(0.1)
        open_msg.points.append(point)

        self.pub_command.publish(open_msg)
        resp = move_gripper_toResponse()
        resp.success = True
        return resp

    def close_gripper(self,req):
        # close_msg = Float64MultiArray()
        # close_msg.data = [0,0]
        # close_msg.layout.data_offset = 0
        # self.pub_command.publish(close_msg)
        open_msg = JointTrajectory()
        open_msg.joint_names = ["gripper_joint_l", "gripper_joint_r"]
        point = JointTrajectoryPoint()
        point.positions = [0, 0]
        point.time_from_start = rospy.Time(1)
        open_msg.points.append(point)

        self.pub_command.publish(open_msg)
        resp = TriggerResponse()
        resp.success = True
        resp.message = "Gripper Closed"
        return resp


if __name__ == "__main__":
    GripperSimul()
    rospy.spin()
