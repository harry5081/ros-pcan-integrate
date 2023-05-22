#!/usr/bin/env python2

import socket
import time 
import json
import pickle

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
import tf


def jsonify(msg):
    tokens = msg.split('\r\n')[0].split('\n')[0]
    return json.loads(tokens)


class rosSocketNode():
    def __init__(self):
        print("This is the script from a ros node as socket client")
        print("Conneting to socket server of pcan ")

        rospy.init_node('Ros_Socket_Communicate_Node')

        self.cmd_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback)

        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=100)

    def cmd_callback(self, received_cmd_msg):

        self.write_msg["VX"] = received_cmd_msg.linear.x*1000 # (mm) from the robot to (m) to ros
        self.write_msg["VY"] = received_cmd_msg.linear.y*1000
        self.write_msg["omega"] = received_cmd_msg.angular.z

        self.client_socket.send(json.dumps(self.write_msg).encode())  # send message
        data = self.client_socket.recv(1024).decode()  # receive response

    def request_robot_states(self):

        self.client_socket.send(json.dumps(self.read_msg).encode())  # send message
        data = self.client_socket.recv(1024).decode()  # receive response
        jdata = jsonify(data)
        
        odom_msg = Odometry()

        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = 'physical_robot_odom';

        odom_msg.pose.pose.position.x = jdata["pos_X"]*1e-3
        odom_msg.pose.pose.position.y = jdata["pos_Y"]*1e-3
        print(odom_msg.pose.pose.position.x)
        print(odom_msg.pose.pose.position.y)
        print()
        # quan = tf.transformations.quaternion_from_euler(0,0, jdata["theta_Z"])
        # odom_quat = tf.Quaternion.createQuaternionMsgFromYaw(jdata["theta_Z"])
        # odom_msg.pose.pose.orientation = odom_quat
        odom_msg.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0, jdata["theta_Z"]))
        self.odom_pub.publish(odom_msg)

    
    def socket_construct(self):
        host = socket.gethostname()  # as both code is running on same pc
        port = 30001  # socket server port number

        self.client_socket = socket.socket()  # instantiate
        self.client_socket.connect((host, port))  # connect to the server
        self.read_msg = {"CMD":"Read"}
        self.write_msg = {"CMD":"Write", "VX": 0, "VY": 0, "omega":0}

    def operate(self):
        node_timer = rospy.Rate(1)
        # try:
        self.socket_construct()
        while not rospy.is_shutdown():
            self.request_robot_states()
            node_timer.sleep()
        # except:
            # print("Fail: socket not connected..")


if __name__ == '__main__':
    ros_socket_node = rosSocketNode()
    try:
        ros_socket_node.operate()
        pass
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
