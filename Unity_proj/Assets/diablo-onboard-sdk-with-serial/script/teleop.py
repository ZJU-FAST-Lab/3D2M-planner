#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String

import time
import sys
import tty
import termios

import threading

keyQueue = []
old_setting = termios.tcgetattr(sys.stdin)

def readchar():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def getKeyBoard():
    global keyQueue
    while True:
        c = readchar()
        keyQueue.append(c)

t1 =threading.Thread(target=getKeyBoard)
t1.setDaemon(True)
t1.start()

def main():
    rospy.init_node('dj_teleop')
    keypub = rospy.Publisher('/DJ_teleop', String, queue_size = 2)
    print("dj teleop start!")
    while True:
        if len(keyQueue) > 0:
            key = keyQueue.pop(0)
        else:
            key = " "
        if key == '1':
            break
        keypub.publish(key)
        time.sleep(0.04)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_setting)
    print('exit!')


if __name__ == '__main__':
    main()
