from JW_control import *
import mujoco_py

xml_path = './MuJoCo_JW/model/scene.xml'

# 모델 로드
m = mujoco_py.load_model_from_path(xml_path)
sim = mujoco_py.MjSim(m)
d = sim.data
viewer = mujoco_py.MjViewer(sim)

robot = JWControl(sim, m, d)
dxl = RemoteDynamixel([5,6,7,8,9])
while True: # 반복 제어랑 원격제어랑 if로 아예 공간 나눠야함
    joint_torq = robot.move_fixed_traj()
    if d.time >= 5:
        joint_torq = robot.go_init_point()

    for i in range(m.nu):
        d.ctrl[i] = joint_torq[i]

    sim.step()
    viewer.render()