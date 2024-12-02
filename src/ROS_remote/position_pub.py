#!/usr/bin/env python
import os
import sys
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../'))

from MuJoCo_JW.dxl import RemoteDynamixel


class RemotePub:
    def __init__(self) -> None:
        self.position_pub = rospy.Publisher('position', Float64MultiArray, queue_size=10)
        self.qval_pub = rospy.Publisher('velocity', Float64MultiArray, queue_size=10)
        self.image_sub = rospy.Subscriber('image', Image, self.image_callback)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(2)
        self.remote_dxl = RemoteDynamixel([5, 6, 7, 8, 9])

        self.q_position = []
        self.received_image = None

    def position_publisher(self) -> None:
        msg = Float64MultiArray()
        self.q_position = self.remote_dxl.get_qpos()
        msg.data = self.q_position
        self.position_pub.publish(msg)

    def velocity_publisher(self) -> None:
        msg = Float64MultiArray()
        msg.data = self.remote_dxl.get_qvel()
        self.qval_pub.publish(msg)

    def image_callback(self, msg) -> None:
        self.received_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

def main():
    rospy.init_node('remote_pub_node')
    remote_pub = RemotePub()
    rate = rospy.Rate(200)

    while not rospy.is_shutdown():
        remote_pub.position_publisher()
        remote_pub.velocity_publisher()

        if remote_pub.received_image is not None:
            # 이미지를 화면에 띄우기
            cv2.imshow('Received Image', remote_pub.received_image)
            # q 키를 눌러 창을 닫을 수 있게 함
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        rate.sleep()

    cv2.destroyAllWindows()  # 모든 OpenCV 창 닫기

if __name__ == '__main__':
    main()