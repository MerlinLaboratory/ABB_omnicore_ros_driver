#!/usr/bin/python
import socket
import struct
import threading
import sys, getopt
import collections
import time
#from python_qt_binding.QtCore import QTimer, QObject, QThread
from threading import Thread
from multiprocessing import Process, Queue, Value, SimpleQueue
import select

HOST ='10.0.0.1' # Symbolic name meaning all available interfaces

class UDPNode(Process):
    """
    UDP node
    - UDPNode(port)
      port of listening
    
      UDPNode.sendto(message, IPaddress, port)
      UDPNode.setMsgCallback(function)
            function type - function(classe, msg)
    
    """
    def __init__(self, port, hostIP):
        Process.__init__(self)
        # inizializza su internet con protocollo UDP
        try:
            self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            print("UDP socket created")
        except socket.error as msg:
            print('Failed to create socket. Error Code : ' + str(msg[0]) + ' Message ' + msg[1])
            self.sock = None


        # self.sock.setblocking(0)
        # self.sock.settimeout(0.001)
        self.port = port
        self._stop = Value('h', 0)
        self.callbackFunction = None
        self.messageList = SimpleQueue()
        self._hostIP = hostIP
        # self._function_queue = q


    def setMsgCallbackFunction(self, fun):
        self.callbackFunction = fun


    def run(self): # non dovrebbe servire mutex per risorsa condivisa socket
        self._stop.value = 0
        if (self.sock != None):
            try:
                self.sock.bind((self._hostIP,self.port))
                while self._stop.value == 0:
                    try:
                        read_list, write_list, _ = select.select([self.sock],[],[],0.1)
                        if len(read_list) >= 1 and self.sock in read_list:
                            d = self.sock.recvfrom(1024)
                            
                            if (self.callbackFunction != None): # se callfun
                                self.callbackFunction(d) # non capisco quel self
                        # self._function_queue.put(d)
                    except socket.timeout: # nel caso non ricevo una minchia
                        pass

                    if not self.messageList.empty():
                        dataSend = self.messageList.get()
                        msg = dataSend[0]
                        addr = dataSend[1]
                        try:
                            self.sock.sendto(msg,addr) # spedisco
                        except socket.error:
                            print("msg" + str(msg) + " not sended")
                    
                self.sock.close() # chiudo a fine while
                print("Socket close")
            except socket.error as msg:
                print('Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1])
                self.sock.close()
        else:
            print("No socket created")
        

    def sendto(self,msg, IPaddress, port):
        self.messageList.put((msg, (IPaddress,port)))

    def stop(self):
        self._stop.value = 1 
            
############################################################################################################


def renderPack(dictionary):
    try:
        return struct.pack('=hhhihidd',dictionary['Header'],
              dictionary['ForceSensor'],
              dictionary['InteractionForce'],
              dictionary['RobotVelocity'],
              dictionary['RobotPosition'],
              dictionary['TargetPosition'],
              dictionary['TaskData'],
              dictionary['AdditionalData'])
    except struct.error as msg:
        print("message not trasformed")
        print(msg)
        raise


def renderUnpack(msg):
    try:
        vett = struct.unpack('=hhhihidd',msg)
        return {'Header':vett[0],
                'ForceSensor':vett[1],
                'InteractionForce':vett[2],
                'RobotVelocity':vett[3],
                'RobotPosition':vett[4],
                'TargetPosition':vett[5],
                'TaskData':vett[6],
                'AdditionalData':vett[7]}
    except struct.error as msg:
        print("message not trasformed")
        print(msg)
        raise
        return ""


################################################################################################################

def fun(md):
    msg = (md[0])
    print("--------------------------------")    
    print("messaggio ricevuto: " + str(msg))
    print("messaggio  arrivato da : " + md[1][0] + ":"+  str(md[1][1]))
    try:
        val = renderUnpack(msg)
        print("messaggio ricevuto ForceSensor: " + str(val['ForceSensor']) +  "   da: " + str(md[1]))
        print("messaggio ricevuto InteractionForce: " + str(val['InteractionForce']) +  "   da: " + str(md[1]))
    except struct.error as msg:
        print("message not trasformed")
        print(msg)
        pass
    except Exception as e:
        print("unexpected error")
        print(e)
        pass
    print("tempo : " + str(time.time()))    


def main(argv):
    print(argv[0])
    if (argv[0] == 'receiver'):
        ser = UDPNode(8889, "127.0.0.1")
        # ser.setMsgCallbackFunction(printRender)
        ser.setMsgCallbackFunction(fun)
        input("parto?")
        ser.start()
        input("stoppo?")
        ser.stop()
    elif (argv[0] == 'sender'):
        ric = UDPNode(8887, "127.0.0.1")
        ric.start()
        

        dictRender = {'Header':1,
                      'ForceSensor':2,
                      'InteractionForce':3,
                      'RobotVelocity':4,
                      'RobotPosition':5,
                      'TargetPosition':6,
                      'TaskData':7,
                      'AdditionalData':8}

        for i in range(100):
            ric.sendto(renderPack(dictRender), '127.0.0.1', 8889)
        
        
        input("stoppo?")
        print("stoppo")
        ric.stop()
        

if __name__ == "__main__":
    main(sys.argv[1:])