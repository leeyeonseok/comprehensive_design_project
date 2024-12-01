#!/usr/bin/env python
import os
import sys
import rospy
from std_msgs.msg import Float64MultiArray

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../'))

from MuJoCo_JW.dxl import RemoteDynamixel


class RemotePub:
    def __init__(self) -> None:
        self.position_pub = rospy.Publisher('positions', Float64MultiArray, queue_size=10)
        self.remote_dxl = RemoteDynamixel([5, 6, 7, 8, 9])

    def position_publisher(self) -> None:
        msg = Float64MultiArray()
        msg.data = self.remote_dxl.get_qpos()
        self.position_pub.publish(msg)


def main():
    rospy.init_node('remote_pub_node')
    remote_pub = RemotePub()
    rate = rospy.Rate(200)

    while not rospy.is_shutdown():
        remote_pub.position_publisher()
        rate.sleep()


if __name__ == '__main__':
    main()
