import pybullet as p
import pybullet_data

import time


p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally

# p.loadURDF("plane.urdf")

# 取消碰撞检测
hand = p.loadURDF("urdf/ex12.urdf", useFixedBase=True, basePosition=[0, 0, 0])
p.setGravity(0, 0, -9.8)


valid_joints = []
for i in range(p.getNumJoints(hand)):
    info = p.getJointInfo(hand, i)
    print(info[1])
    if info[2] == p.JOINT_REVOLUTE:
        
        valid_joints.append(i)


print(valid_joints)
# 增加三个xyz的拖动条

tx = p.addUserDebugParameter(f"TX", 0, 0.3, 0.02)
ty = p.addUserDebugParameter(f"TY", 0, 0.3, 0.06)
tz = p.addUserDebugParameter(f"TZ", 0., 0.6, 0.1)

sliders = [tx, ty, tz]

# 逆运动学
while True:
    xyz = [p.readUserDebugParameter(i) for i in sliders]

    # 计算逆运动学
    joint_positions = p.calculateInverseKinematics(hand, 14, xyz, maxNumIterations=300, residualThreshold=1e-4)

    joint_positions = [0]*12

    # print(len(joint_positions))
    # 设置关节位置
    for i, joint_position in zip(valid_joints, joint_positions):
        p.setJointMotorControl2(hand, i, p.POSITION_CONTROL, joint_position)
    
    p.stepSimulation()

    xyz_now5 = p.getLinkState(hand, 14, computeForwardKinematics=1)[4]
    # xyz_now5 = p.getLinkState(hand, 3, computeForwardKinematics=1)[0]

    print(f'target: {xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}, now5: {xyz_now5[0]:.3f}, {xyz_now5[1]:.3f}, {xyz_now5[2]:.3f}')

    time.sleep(1./240.)
# while True:
#     p.stepSimulation()
#     time.sleep(1./240.)

