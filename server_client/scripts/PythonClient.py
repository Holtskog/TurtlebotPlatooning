#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import socket
import struct
import csv

# Class for requesting data from the server:
class Speed():
    def __init__(self, host_ip=None, port=None):
        # Set up host's ip and port number:
        if host_ip is None and port is None:
            self.host_ip = '172.20.10.7'
            self.port = 50018
        else:
            self.host_ip = host_ip
            self.port = port

    def send_req(self):
        # Declare initial message:
        self.initial_message = 'speed'
        # open the connection:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.host_ip, self.port))
        # Send the data:
        self.sock.sendall(str(self.initial_message)+'\\r\\n')
        # Save and print response from server:
        response = self.sock.recv(64)
        response = response.split('\\r\\n')[0]
        response = float(response)
        # Close the connection:
        self.sock.close()
        return response

    def send_close(self):
        self.closing_message = 'close'

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.host_ip, self.port))

        self.sock.sendall(self.closing_message + '\\r\\n')

        self.sock.close()



# Function for publishing the speed to a topic
def add_speed():
    pub = rospy.Publisher("lead_speed", Float32, queue_size=1000)
    speed = Speed()
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        msg = speed.send_req()
        pub.publish(msg)
        rospy.loginfo("%s" % msg)
        rate.sleep()
    speed.send_close()



if __name__ == "__main__":
    rospy.init_node("speed_client", anonymous=False)
    try:
        add_speed()
    except rospy.ROSInterruptException:
        pass
