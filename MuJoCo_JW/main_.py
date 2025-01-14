#!/usr/bin/env python 
import os, sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import rospy
from JW_control import *
import mujoco_py
from std_msgs.msg import Float64MultiArray, Bool, Int16
from dxl import *
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


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
        self.mistake_sub = rospy.Subscriber('mistake', Bool, self.mistake_callback)
        self.remote_sub = rospy.Subscriber('remote', Bool, self.remote_callback)
        self.image_pub = rospy.Publisher('image', Image, queue_size=10)
        self.torque_pub = rospy.Publisher('torque', Int16, queue_size=1)
        self.qpos_d = [0, 0, 0, 0, 0]
        self.qvel_d = [0, 0, 0, 0, 0]
        self.remote_mode = False
        self.mistake_mode = False
        self.bridge = CvBridge()
        self.camera_indices = [2,3,4]
        self.cap = None
        self.initialize_camera()
        self.step = None
        self.flag = False
        self.mistake_flag = False
            

    def initialize_camera(self):
        """동적으로 카메라를 초기화하여 연결."""
        for index in self.camera_indices:
            cap = cv2.VideoCapture(index)
            if cap.isOpened():
                rospy.loginfo(f"Camera connected on index {index}")
                self.cap = cap
                return
            cap.release()

    def position_callback(self, msg):
        self.qpos_d = msg.data

    def velocity_callback(self, msg):
        self.qvel_d = msg.data

    def remote_callback(self, msg):
        self.remote_mode = msg.data
    
    def mistake_callback(self, msg):
        self.mistake_mode = msg.data

    def image_publisher(self) -> None:
        ret, frame = self.cap.read()  # 프레임 읽기
        fish_eye_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_pub.publish(fish_eye_image)
    
    def torque_publisher(self, torque):
        self.torque_pub.publish(torque)

def main():
    mj = MuJoCo()
    robot = mj.robot
    rospy.init_node('main_node')
    gripper_pos = 0
    AT = Remote_contorl()
    dxl = MainDynamixel([11, 12, 13, 14, 15, 16])
    while not rospy.is_shutdown(): # 반복 제어랑 원격제어랑 if로 아예 공간 나눠야함
        if AT.remote_mode is True and robot.complete_a_cycle is True:
            # ======================================= REMOTE MODE ==============================================
            gripper_torque = dxl.get_gripper_torque()
            AT.torque_publisher(gripper_torque)
            
            joint_torq = robot.remote_control(AT.qpos_d[:-1], AT.qvel_d[:-1])
            gripper_pos = AT.qpos_d[-1] * 2048 / pi
            AT.image_publisher()
            dxl.control_by_remote(mj.d.qpos)
            if gripper_pos > 1000:
                gripper_pos = 1000
            dxl.control_gripper(gripper_pos)
        else:
            # ======================================= RULE BASED MODE ==========================================
            if robot.complete_a_cycle:
                # initialize the class
                mj.robot = JWControl(mj.sim, mj.m, mj.d, mj.d.time)
                robot = mj.robot
                AT.mistake_flag = False
                time.sleep(3)

            # ======================================= MISTAKE DECISION ==========================================
            points = robot.points
            
            if AT.mistake_mode == True and robot.step == 13 and AT.mistake_flag == False:
                gripper_pos = 0
                dxl.control_gripper(gripper_pos)
                AT.mistake_flag = True

            joint_torq = robot.move_fixed_traj(points)

            # ================================= INITIALIZE POSITION OF THE ROBOT =================================
            if mj.d.time > robot.traj_time[-1]:
                joint_torq = robot.go_init_point()

            # ======================================== GRIPPER CONTROL ===========================================
            if robot.step == 3 and AT.flag is False:
                gripper_pos = 600
                dxl.control_gripper(gripper_pos)
                AT.flag = True

            if robot.step == 14 and AT.flag is True:
                gripper_pos = 0
                dxl.control_gripper(gripper_pos)
                AT.flag = False

            dxl.control_pos(mj.d.qpos)

        for i in range(mj.m.nu):
            mj.d.ctrl[i] = joint_torq[i]

        mj.sim.step()
        mj.viewer.render()
    
    dxl.close_port()

if __name__=='__main__':
    main()
