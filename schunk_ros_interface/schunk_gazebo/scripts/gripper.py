#!/usr/bin/env python

######################### IMPORTANT PLASE READ #########################
# This file is used only as a mockup to simulate the services exposed by 
# the real gripper. Indeed this file is thought to be launched only when 
# simulating the gripper in Gazebo/Rviz. Do not launch it with the real
# hardware
######################### IMPORTANT PLASE READ #########################

from schunk_interfaces.srv import JogTo, Release, SimpleGrip, JogToResponse, ReleaseResponse, SimpleGripResponse
import rospy

from std_msgs.msg import Float64MultiArray


class GripperMockup:
    
  def __init__(self):

    # Mockup services
    self.jot_to_srv = rospy.Service('jog_to', JogTo, self.jog_to_cb)
    self.release_srv = rospy.Service('release', Release, self.release_cb)
    self.simple_grip_srv = rospy.Service('simple_grip', SimpleGrip, self.simple_grip_cb)

    # Publisher
    self.command_pub = rospy.Publisher('gripper_controller/command', Float64MultiArray, queue_size=10)

  def jog_to_cb(self, req):
    self.move_gripper(req.position)
    rospy.sleep(0.5)
    
    res = JogToResponse()
    res.success = True
    return res

  def release_cb(self, req):
    self.move_gripper(0.0)
    rospy.sleep(0.5)
    
    res = ReleaseResponse()
    res.success = True
    return res
      
  def simple_grip_cb(self, req):
    self.move_gripper(0.4)
    rospy.sleep(0.5)
    
    res = SimpleGripResponse()
    res.success = True
    return res

  def move_gripper(self, position):
    msg = Float64MultiArray()
    msg.data.append(position)
    msg.data.append(position)
    
    self.command_pub.publish(msg)
    
    return True


if __name__ == "__main__":
    
    rospy.init_node('gripper_mockup')
    GripperMockup()
    rospy.spin()
    