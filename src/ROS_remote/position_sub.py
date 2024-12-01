#!/usr/bin/env python
import os
import sys
import rospy
from std_msgs.msg import Float64MultiArray

# 현재 파일의 경로를 기준으로 MuJoCo_JW 경로 추가
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../'))

from MuJoCo_JW.dxl import RemoteDynamixel


class RemoteSub:
    def __init__(self) -> None:
        self.position_sub = rospy.Subscriber('positions', Float64MultiArray, self.position_callback)

    def position_callback(self, msg) -> None:
        print(msg.data)


def main():
    rospy.init_node('remote_sub_node')  # ROS 노드 초기화
    remote_sub = RemoteSub()
    rate = rospy.Rate(200)

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()
