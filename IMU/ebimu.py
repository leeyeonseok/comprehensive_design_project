import sys 
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
project_dir = os.path.join(current_dir, "..")  # 상위 디렉토리로 이동
sys.path.append(project_dir)
project_dir = os.path.join(project_dir, "MuJoCo_JW")
sys.path.append(project_dir)
import serial
import numpy as np
from MuJoCo_JW.functions import euler_to_quaternion

class EBIMU:
    def __init__(self):
        comport_num = "/dev/ttyUSB0"
        comport_baudrate = 115200
        self.ser = serial.Serial(port=comport_num,baudrate=comport_baudrate)
        self.init_state = self.get_init()

    def get_init(self):
        while 1:
            line = self.ser.readline()
            line = line.decode('utf-8')
            words = line.split(",")

            if(-1 < words[0].find('*')) :
                words[0]=words[0].replace('*','')

            init_rpy = np.array([float(word) for word in words[:3]])
            # for word in words:
            #     print(word + ' ', end='')
            if init_rpy.any() != 0:
                break
        return init_rpy

    def get_data(self):
        line = self.ser.readline()
        line = line.decode('utf-8')
        words = line.split(",")

        if(-1 < words[0].find('*')) :
            words[0]=words[0].replace('*','')

        rpy = np.array([float(word) for word in words[:3]]) - self.init_state
        # for word in words:
        #     print(word + ' ', end='')
        
        quaternion = euler_to_quaternion(rpy[0], rpy[1], rpy[2])
        return quaternion

    def close(self):
        self.ser.close()

a = EBIMU()
a.get_data()