#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def main():
    # ROS 초기화
    rospy.init_node('camera_publisher', anonymous=True)
    pub = rospy.Publisher('camera/image', Image, queue_size=10)
    bridge = CvBridge()
    rate = rospy.Rate(200)
    # 카메라 열기
    cap = cv2.VideoCapture(2)  # 0은 기본 카메라 장치

    while not rospy.is_shutdown():
        ret, frame = cap.read()  # 프레임 읽기
        if not ret:
            break

        # OpenCV 이미지를 ROS 이미지 메시지로 변환
        ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        # 이미지 퍼블리시
        pub.publish(ros_image)

        # ROS의 주기 설정
        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


# import cv2

# # 카메라 열기
# cap = cv2.VideoCapture(2)  # 0은 기본 카메라 장치

# while True:
#     ret, frame = cap.read()  # 프레임 읽기
#     if not ret:
#         break

#     cv2.imshow('Camera', frame)  # 프레임 표시

#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# cap.release()
# cv2.destroyAllWindows()
