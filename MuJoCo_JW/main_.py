#!/usr/bin/env python 
import os, sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import rospy
from JW_control import *
import mujoco_py
from std_msgs.msg import Float64MultiArray
from dxl import *
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


# cur_dir = os.getcwd()
# target_dir = os.path.join(cur_dir, '/comprehensive_design_project/src/ROS_remote/')
# print(target_dir)
# sys.path.append(target_dir)

# from position_sub import cur_pos


class MuJoCo:
    def __init__(self):
        self.xml_path = '/home/acewnd/catkin_ws/src/mujoco_ros_sim/src/model/scene.xml'
        self.m = mujoco_py.load_model_from_path(self.xml_path)
        self.sim = mujoco_py.MjSim(self.m)
        self.d = self.sim.data
        self.viewer = mujoco_py.MjViewer(self.sim)        
        self.robot = JWControl(self.sim, self.m, self.d)

class Remote_contorl:
    def __init__(self):
        self.position_sub = rospy.Subscriber('position', Float64MultiArray, self.position_callback)
        self.velocity_sub = rospy.Subscriber('velocity', Float64MultiArray, self.velocity_callback)
        self.image_pub = rospy.Publisher('image', Image, queue_size=10)
        self.qpos_d = [0, 0, 0, 0, 0]
        self.qvel_d = [0, 0, 0, 0, 0]
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(2)


    def position_callback(self, msg):
        # print(msg.data)
        self.qpos_d = msg.data

    def velocity_callback(self, msg):
        # print(msg.data)
        self.qvel_d = msg.data

    # def image_publisher(self) -> None:
    #     # msg = Image()
    #     ret, frame = self.cap.read()  # 프레임 읽기
    #     fish_eye_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
    #     # msg.data = fish_eye_image
    #     self.image_pub.publish(fish_eye_image)


def main():
    mj = MuJoCo()
    robot = mj.robot
    # rate = rospy.Rate(200)
    rospy.init_node('main_node')
    
    AT = Remote_contorl()
    dxl = MainDynamixel([11, 12, 13, 14, 15])
    while not rospy.is_shutdown(): # 반복 제어랑 원격제어랑 if로 아예 공간 나눠야함
        joint_torq = robot.move_fixed_traj(AT.qpos_d, AT.qvel_d)
        
        # AT.image_publisher()

        # if d.time >= 5:
        #     joint_torq = robot.go_init_point()

        for i in range(mj.m.nu):
            mj.d.ctrl[i] = joint_torq[i]

        dxl.control_pos(mj.d.qpos)

        mj.sim.step()
        mj.viewer.render()
        # rate.sleep()
    
    #dxl.close_port()

if __name__=='__main__':
    main()
