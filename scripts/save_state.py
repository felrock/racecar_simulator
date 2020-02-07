#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
import threading
import time

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from racecar_simulator.msg import carstate

class TreeSearch:
    pass

class SaveState:
    """

    """
    def __init__(self):

        # Topic names
        read_state_topic  = '/read_state'
        write_state_topic = '/write_state'
        update_topic = '/update'
        drive_topic  = '/drive'
        scan_topic   = '/scan'

        # State & booleans
        self.first = True
        self.first_state = None
        self.root_state = None
        self.latest_state = None

        # Publishers & Subscribers
        self.read_state_sub = rospy.Subscriber(read_state_topic,
                                          carstate,
                                          self.stateCallback)

        self.scan_topic = rospy.Subscriber(scan_topic,
                                           LaserScan,
                                           self.scanCallback)

        self.write_state_pub = rospy.Publisher(write_state_topic,
                                                carstate,
                                                queue_size=1)

        self.drive_pub = rospy.Publisher(drive_topic,
                                          AckermannDriveStamped,
                                          queue_size=1)

        self.update_pub = rospy.Publisher(update_topic,
                                          String,
                                          queue_size=1)

    def stateCallback(self, msg):
        """

        """

        if msg:
            if self.first:
                self.first_state = msg
                self.first = False
                print(msg.velocity)
            self.latest_state = msg

    def scanCallback(self, msg):
        """

        """

        self.latest_state = msg

    def drive(self, speed, angle):
        """

        """

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

    def writeState(self, state):
        """

        """

        self.write_state_pub.publish(state)

def main():
    """
        Main-loop, here algorithms should be run
    """

    ss = SaveState()
    time.sleep(0.5)
    ss.update_pub.publish(String('bajs'))
    time.sleep(0.5)
    ss.drive(0.5, 0.5)

    for _ in range(1000):

        time.sleep(0.01)
        ss.update_pub.publish(String('bajs'))

    print(ss.first_state)
    ss.writeState(ss.first_state)
    for _ in range(10):

        ss.update_pub.publish(String('bajs'))


def start(args):
    """
        Run all ros necessary things..
    """

    rospy.init_node("SaveState_node", anonymous=True)
    thread = threading.Thread(target=main)
    rospy.sleep(0.1)
    thread.start()
    rospy.spin()

if __name__=='__main__':
    start(sys.argv)
