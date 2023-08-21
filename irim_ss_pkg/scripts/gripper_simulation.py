#!/usr/bin/env python

from std_srvs.srv import TriggerRequest,TriggerResponse,Trigger


from std_msgs.msg import Float64MultiArray
import rospy

class GripperSimul:

    def __init__(self):
        rospy.init_node('gripper_simulation')
        self.grip_in  = rospy.Service('/grip_in', Trigger, self.close_gripper)
        self.grip_out = rospy.Service('/grip_out', Trigger, self.open_gripper)
        self.pub_command       = rospy.Publisher("/gripper_controller/command", Float64MultiArray, queue_size=1,latch=True)


    def open_gripper(self,req):
        open_msg = Float64MultiArray()
        open_msg.data = [50,50]
        open_msg.layout.data_offset = 0
        self.pub_command.publish(open_msg)
        resp = TriggerResponse()
        resp.success = True
        resp.message = "Gripper Opened"
        return resp

    def close_gripper(self,req):
        close_msg = Float64MultiArray()
        close_msg.data = [-50,-50]
        close_msg.layout.data_offset = 0
        self.pub_command.publish(close_msg)
        resp = TriggerResponse()
        resp.success = True
        resp.message = "Gripper Closed"
        return resp


if __name__ == "__main__":
    GripperSimul()
    rospy.spin()
