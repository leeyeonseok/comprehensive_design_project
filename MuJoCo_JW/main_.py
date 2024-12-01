from JW_control import *
import mujoco_py

xml_path = './MuJoCo_JW/model/scene.xml'

# 모델 로드
m = mujoco_py.load_model_from_path(xml_path)
sim = mujoco_py.MjSim(m)
d = sim.data
viewer = mujoco_py.MjViewer(sim)

robot = JWControl(sim, m, d)

while True:
    if d.time <= 20:
        joint_torq = robot.move_fixed_traj()
    else:
        joint_torq = robot.go_init_point() 

    for i in range(m.nu):
        d.ctrl[i] = joint_torq[i]

    sim.step()
    viewer.render()