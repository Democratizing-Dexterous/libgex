import pybullet as p
import os.path as osp
import numpy as np
abs_path = osp.dirname(osp.abspath(__file__))



class KinEX12:
    def __init__(self) -> None:
        self.name = 'EX12'
        p.connect(p.DIRECT)
        self.offset_z = 3

        self.bullet_hand = p.loadURDF(osp.join(abs_path, "urdf/ex12.urdf"), useFixedBase=True, basePosition=[0, 0, self.offset_z])
        for i in range(20):
            p.stepSimulation()

        self.thumb_link_id = 4
        self.thumb_joint_ids = [0, 1, 2, 3]

        self.index_link_id = 9
        self.index_joint_ids = [5, 6, 7, 8]

        self.middle_link_id = 14
        self.middle_joint_ids = [10, 11, 12, 13]

        self.direction_finger1 = [1, -1, -1, -1]
        self.direction_finger2 = [-1, 1, 1, 1]
        self.direction_finger3 = [-1, -1, -1, -1]

    def fk_finger1(self, q=[0]*4):
        """
        finger1 正运动学，3自由度
        """
        # 匹配关节方向，degree to rad
        q = [q_*d*np.pi/180 for q_, d in zip(q, self.direction_finger1)]


        for i, joint_position in zip(self.thumb_joint_ids, q):
            p.setJointMotorControl2(self.bullet_hand, i, p.POSITION_CONTROL, joint_position)
        
        for i in range(20):
            p.stepSimulation()

        ee_pos = p.getLinkState(self.bullet_hand, self.thumb_link_id, computeForwardKinematics=1)[4]

        ee_pos = list(ee_pos)
        ee_pos[2] -= self.offset_z

        return ee_pos
    
    def ik_finger1(self, xyz):
        # 计算逆运动学
        joint_positions = p.calculateInverseKinematics(self.bullet_hand, self.thumb_link_id, xyz, maxNumIterations=300, residualThreshold=1e-4)

        q = joint_positions[:4]
        # 匹配关节方向，rad to degree
        q = [q_*d*180/np.pi for q_, d in zip(q, self.direction_finger1)]
        
        return q


    def fk_finger2(self, q=[0]*4):
        """
        finger2 正运动学，4自由度
        """
        # 匹配关节方向，degree to rad
        q = [q_*d*np.pi/180 for q_, d in zip(q, self.direction_finger2)]


        for i, joint_position in zip(self.index_joint_ids, q):
            p.setJointMotorControl2(self.bullet_hand, i, p.POSITION_CONTROL, joint_position)
        
        for i in range(20):
            p.stepSimulation()

        ee_pos = p.getLinkState(self.bullet_hand, self.index_link_id, computeForwardKinematics=1)[4]

        ee_pos = list(ee_pos)
        ee_pos[2] -= self.offset_z

        return ee_pos
    
    def ik_finger2(self, xyz):
        # 计算逆运动学
        joint_positions = p.calculateInverseKinematics(self.bullet_hand, self.index_link_id, xyz, maxNumIterations=300, residualThreshold=1e-4)

        q = joint_positions[4:8]

        q = [q_*d*180/np.pi for q_, d in zip(q, self.direction_finger2)]
        
        return q
    
    def fk_finger3(self, q=[0]*4):
        """
        finger3 正运动学，4自由度
        """
        # 匹配关节方向，degree to rad
        q = [q_*d*np.pi/180 for q_, d in zip(q, self.direction_finger3)]


        for i, joint_position in zip(self.middle_joint_ids, q):
            p.setJointMotorControl2(self.bullet_hand, i, p.POSITION_CONTROL, joint_position)
        
        for i in range(20):
            p.stepSimulation()

        ee_pos = p.getLinkState(self.bullet_hand, self.middle_link_id, computeForwardKinematics=1)[4]

        ee_pos = list(ee_pos)
        ee_pos[2] -= self.offset_z

        return ee_pos
    
    def ik_finger3(self, xyz):
        # 计算逆运动学
        joint_positions = p.calculateInverseKinematics(self.bullet_hand, self.middle_link_id, xyz, maxNumIterations=300, residualThreshold=1e-4)

        q = joint_positions[8:12]
        # 匹配关节方向，rad to degree
        q = [q_*d*180/np.pi for q_, d in zip(q, self.direction_finger3)]
        
        return q

if __name__ == "__main__":
    kin = KinGX11()
