#!/usr/bin/env python

import random
import os
import rospkg
import time

import rospy
import tf2_ros
import gazebo_msgs.msg
import geometry_msgs.msg
import time
import math
import tf
from std_msgs.msg import Float64MultiArray, Int16
import threading
import random
import numpy as np
from tf.transformations import quaternion_matrix,euler_from_quaternion,quaternion_from_euler, euler_from_matrix

np.set_printoptions(suppress=True)


# time.sleep(15)

rp = rospkg.RosPack()
path = rp.get_path('irim_ss_pkg')

urdf_file = path + '/xacro/Extra/dice.urdf'

# print(urdf_file)

rx = random.uniform( 0.0, 0.1)
ry = random.uniform(-0.15, 0.15)

RA = random.uniform(-2.8, 2.8)
RB = random.uniform(-2.8, 2.8)
RC = random.uniform(-2.8, 2.8)


cmd = "rosrun gazebo_ros spawn_model -urdf -file " + urdf_file + " -model dice -z 0.9 -x " + str(rx) + " -y " + str(ry) + " -R " + str(RA)  + " -P " + str(RB)  + " -Y " + str(RC) 

print(cmd)

os.system(cmd)

class dice_ros:

    def __init__(self):
        self.broadcaster       = tf.TransformBroadcaster()
        self.listener          = tf.TransformListener()
        self.rate              = rospy.Rate(10.0)
        self.sub               = rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, self.callback_dice)
        self.pub_fake          = rospy.Publisher('force_sensor_value', Float64MultiArray, queue_size=1)
        self.pub_number        = rospy.Publisher('dice_value', Int16, queue_size=1)
        self.pub_pose          = rospy.Publisher('dice_pose' , geometry_msgs.msg.PoseStamped, queue_size=1)
        self.last_message      = Float64MultiArray()
        self.channels  = 11
        self.last_message.data = [None] * self.channels
        self.gain      = 100
        self.threshold = 15000
        self._transform = None
        self.mutex = threading.Lock()
        self.msg_dice = Int16()
        self.msg_pose = geometry_msgs.msg.PoseStamped()
        self.dice_R = np.eye(4)


    def callback_dice(self,data):
        for i in range(len(data.name)):
            if data.name[i]=="dice":
                self.mutex.acquire()
                self._transform = geometry_msgs.msg.TransformStamped()
                self._transform.header.stamp = rospy.Time.now()
                self._transform.header.frame_id = "world"
                self._transform.child_frame_id = data.name[i]
                self._transform.transform.translation.x = data.pose[i].position.x
                self._transform.transform.translation.y = data.pose[i].position.y
                self._transform.transform.translation.z = data.pose[i].position.z
                self._transform.transform.rotation.w    = data.pose[i].orientation.w
                self._transform.transform.rotation.x    = data.pose[i].orientation.x
                self._transform.transform.rotation.y    = data.pose[i].orientation.y
                self._transform.transform.rotation.z    = data.pose[i].orientation.z

                self.dice_R = quaternion_matrix([data.pose[i].orientation.x,data.pose[i].orientation.y,data.pose[i].orientation.z,data.pose[i].orientation.w])
                R_row3 = self.dice_R[2,:].round()
                # print(R_row3)
                if R_row3[0] == 1:
                    self.msg_dice.data = 2
                elif R_row3[0] == -1:
                    self.msg_dice.data = 5
                elif R_row3[1] == 1:
                    self.msg_dice.data = 1
                elif R_row3[1] == -1:
                    self.msg_dice.data = 6
                elif R_row3[2] == 1:
                    self.msg_dice.data = 4
                elif R_row3[2] == -1:
                    self.msg_dice.data = 3

                self.mutex.release()
                



    def fake_grasp(self):
        while not rospy.is_shutdown():
            rot_z = 0
            if self._transform != None:
                self.mutex.acquire()
                self.broadcaster.sendTransform((self._transform.transform.translation.x,
                                                self._transform.transform.translation.y,
                                                self._transform.transform.translation.z),
                                                (self._transform.transform.rotation.x,
                                                self._transform.transform.rotation.y,
                                                self._transform.transform.rotation.z,
                                                self._transform.transform.rotation.w),
                                                rospy.Time.now(),
                                                self._transform.child_frame_id,
                                                "world")
                self.pub_number.publish(self.msg_dice)

                # if type(self.dice_R) != NoneType:
                [rx,ry,rz] = euler_from_matrix(self.dice_R, axes='sxyz')
                # else:
                #     rx = 0
                #     ry = 0
                #     rz = 0
                
                qt  = []

                if self.msg_dice.data == 1:
                    rot_z = rz #+ np.pi  # y alto 
                elif self.msg_dice.data == 2:
                    rot_z = np.sign(np.arcsin(self.dice_R[1,1])) * np.arccos(self.dice_R[0,1]) #+ np.pi # x alto 
                elif self.msg_dice.data == 3:
                    rot_z = rz #+ np.pi # z bassa
                elif self.msg_dice.data == 4:
                    rot_z = rz # z alta
                elif self.msg_dice.data == 5:
                    rot_z = np.sign(np.arcsin(self.dice_R[1,1])) * np.arccos(self.dice_R[0,1]) #+ np.pi # x alto 
                elif self.msg_dice.data == 6:
                    rot_z = rz # y alto
                

                while rot_z < 0:
                    rot_z = rot_z + 2*np.pi

                while rot_z > np.pi/4:
                    rot_z = rot_z - np.pi/2

                rot_z = rot_z - np.pi/2

                qt  = quaternion_from_euler(0,3.14,rot_z,axes="sxyz")
                
                self.mutex.release()
                 
            try:
                (trans_grasp,rot_grasp) = self.listener.lookupTransform('/dice', '/grasp_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            try:
                (trans_fing,rot_fing) = self.listener.lookupTransform('/dice', '/gripper_finger_l', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue


            try:
                (trans_dice,rot_dice) = self.listener.lookupTransform('/single_yumi_base_link', '/dice', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            #Force dice z axis to look 
            # eul = euler_from_quaternion([rot_dice[0],rot_dice[1],rot_dice[2],rot_dice[3]],axes="sxyz")
            

            self.msg_pose.header.frame_id  = "single_yumi_base_link"
            self.msg_pose.header.stamp     = rospy.Time.now()
            self.msg_pose.pose.position.x    = trans_dice[0]
            self.msg_pose.pose.position.y    = trans_dice[1]
            self.msg_pose.pose.position.z    = trans_dice[2]
            self.msg_pose.pose.orientation.x = qt[0]
            self.msg_pose.pose.orientation.y = qt[1]
            self.msg_pose.pose.orientation.z = qt[2]
            self.msg_pose.pose.orientation.w = qt[3]
            self.pub_pose.publish(self.msg_pose)

            dist_grasp = math.sqrt((trans_grasp[0]**2) + (trans_grasp[1]**2) + (trans_grasp[2]**2))
            # print("dist grasp:" , dist_grasp)
            if (dist_grasp < 0.02):
                for i in range(self.channels):
                    dist = math.sqrt((trans_fing[0]**2) + (trans_fing[1]**2))
                    self.last_message.data[i] = (1/dist) * self.gain + random.uniform(0.0, 1000.0)
                    # print("dist :" , dist)
            else:
                for i in range(self.channels):
                    self.last_message.data[i] = random.uniform(0.0, 1000.0)



            self.pub_fake.publish(self.last_message)

            self.rate.sleep()




def main():
    rospy.init_node('dice_ros')
    d = dice_ros()
    d.fake_grasp()
    # rospy.spin()

if __name__ == '__main__':
    main()