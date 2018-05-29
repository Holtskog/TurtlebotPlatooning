#!/usr/bin/env python
import rospy
import socket
from sensor_msgs.msg import JointState
import csv

def send_speed():
    HOST = '172.20.10.7' #depending on the robot the IP can change
    PORT = 50018 #port number for the server
    
    #create a connection to which the client can connect
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind((HOST, PORT))
    
    data = 'None'

    while data != 'close':
        #get data from connected client
        sock.listen(1)
        client, address = sock.accept()
        data = client.recv(64)
        data = data.split('\\r\\n')[0]
        
        #obtain the velocity data from the Dynamixel motors
        msg = rospy.wait_for_message("/joint_states", JointState)
        speed = ((msg.velocity[0] + msg.velocity[1])/2)*0.033
        
        #check if the correct keyword was sent
        if data == 'speed':
            #send the velocity data
            client.sendall(str(speed) + '\\r\\n')
    
    rospy.loginfo("Closing server ...")      

if __name__ == "__main__":
    #initialise the node
    rospy.init_node('speed_server', anonymous=False)
    try:
        send_speed()
    except rospy.ROSInterruptException:
        pass