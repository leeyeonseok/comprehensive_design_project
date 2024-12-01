import os, sys
import rospy
from std_msgs.msg import Int16MultiArray

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/MuJoCo_JW")

from MuJoCo_JW.dxl import RemoteDynamixel



class RemotePub:
    def __init__(self) -> None:
        self.position_pub = rospy.Publisher('positions', Int16MultiArray, queue_size=10)
        self.remote_dxl = RemoteDynamixel()

    def position_publisher(self) -> None:
        msg = Int16MultiArray()
        rad_list = self.remote_dxl.get_qpos()
        msg.data = self.remote_dxl.control_pos(rad_list)
        self.position_pub.publish(msg)


def main():
    rate = rospy.Rate(200)
    while rospy.is_shutdown():
        remote_pub = RemotePub()
        remote_pub.position_publisher()
        rate.sleep()


if __name__=='__main__':
    main()
