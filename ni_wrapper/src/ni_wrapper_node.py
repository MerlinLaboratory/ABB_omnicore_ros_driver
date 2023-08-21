#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from server_listener import UDPNode
import struct
import multiprocessing #import Process, Queue, Value, SimpleQueue
import time
import codecs

class NIWrapper():
    def __init__(self):
        if rospy.has_param("~communication/IP"):
            self._ip = rospy.get_param("~communication/IP")
        else:
            raise Exception("No IP param")

        if rospy.has_param("~communication/port"):
            self._port = rospy.get_param("~communication/port")
        else:
            raise Exception("No port param")

        self._last_message = Float64MultiArray()
        self._last_message.data = [0]
        self._message_queue = multiprocessing.Queue()
        self._udp_node = UDPNode(self._port, self._ip)
        self._udp_node.setMsgCallbackFunction(self.cbk_message)
        self._udp_node.start()
        self._pub = rospy.Publisher('force_sensor_value', Float64MultiArray, queue_size=10)


    def cbk_message(self, m):
        # print(m[0].decode(b, 'utf-8'))
        vett = codecs.decode(m[0], 'utf-8').split("\r\n")[0].split("\t")
        self._message_queue.put(vett)
 
    def update(self):   
        res = None
        while not self._message_queue.empty():
            res = self._message_queue.get()
            self._last_message.data = [float(ii) for ii in res]
 
        self._pub.publish(self._last_message)


    def stop(self):
        self._udp_node.stop()

if __name__ == '__main__':
    rospy.init_node('ni_wrapper', anonymous=True)
    # ni = NIWrapper("172.25.114.19",61557)
    ni = NIWrapper()
    rate = rospy.Rate(100) # 100hz

    while not rospy.is_shutdown():
        ni.update()
        rate.sleep()

    rospy.loginfo("STOP")
    ni.stop()
