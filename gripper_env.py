import json
import os

import numpy as np
import pybullet as p
import time

from env_base import BaseEnv

class GripperEnv(BaseEnv):
    def __init__(self,
                 robot_folders,
                 robot_dir,
                 render,
                 tol=0.02,
                 train=True,
                 with_kin=None):
        super().__init__(robot_folders=robot_folders,
                         robot_dir=robot_dir,
                         render=render,
                         tol=tol,
                         train=train,
                         with_kin=with_kin)

    

    def reset(self, robot_id=None): 
        if robot_id is None:
            self.robot_id = np.random.randint(0, self.train_robot_num, 1)[0]
        else:
            self.robot_id = robot_id
        self.reset_robot(self.robot_id)

        ob = self.get_obs()
        self.ep_reward = 0
        self.ep_len =  0
        return ob
        
    def step(self, action):
        scaled_action = self.scale_action(action)
        
        hand_pose_1 = np.array(p.getLinkState(self.sim, 7)[4])

        #action for ur5
        #hand_pose_2 =np.array(p.getBasePositionAndOrientation(self.cube)[0])

        hand_pose= (hand_pose_1)*1
        if .575 <= hand_pose[0] + scaled_action[0] <= .725:
            hand_pose[0] += scaled_action[0]
        if -.15 <= hand_pose[1] + scaled_action[1] <= .15:
            hand_pose[1] += scaled_action[1]
        if (.1 - self.finger_height_offset) <= hand_pose[2] + scaled_action[2] <= (.375 - self.finger_height_offset):
            hand_pose[2] += scaled_action[2]
        #print(hand_pose)
        desired_joint_positions = p.calculateInverseKinematics(
                                            self.sim, self.end_factor,
                                            hand_pose,[1,1,0,0],
                                            lowerLimits=self.ll, upperLimits=self.ul, residualThreshold=1e-5 )[:6]



        #action for finger1
        finger_index_1=self.act_joint_indices[6]
        gripper_pos_1 = p.getJointState(self.sim,finger_index_1)[0]
        gripper_act_1 = np.array([gripper_pos_1 + action[3]])*5 * np.ones(len(self.act_joint_indices[6:8]))
        
        

        #action for finger2
        finger_index_2=self.act_joint_indices[8]
        gripper_pos_2 = p.getJointState(self.sim,finger_index_2)[0]
        gripper_act_2 = np.array([gripper_pos_2 + action[4]])*5 * np.ones(len(self.act_joint_indices[8:]))
        
        desired_joint_positions = np.concatenate([desired_joint_positions, gripper_act_1,  gripper_act_2 ])
        
        #torque
        torque_1=np.ones(6, dtype = None)*10000
        torque_2=np.ones(len(self.act_joint_indices)-6, dtype = None)*17.5
        torque = np.concatenate([torque_1,torque_2])
        
        #torque= 2
        p.setJointMotorControlArray(
            bodyIndex=self.sim,
            jointIndices=self.act_joint_indices,
            controlMode=p.POSITION_CONTROL,
            targetPositions=desired_joint_positions,
            forces=torque
        )

        #robotiq 2f paralel finger 
        if 'robotiq_2f_140' in self.robot_name:
            desire_velocity=[]
            paralel_joint=[16, 17]
            
            for x in range(6, 8):
                desire_velocity.append((p.getJointState(bodyUniqueId=self.sim,
                    jointIndex=self.act_joint_indices[x])[1]))
            
            
            desire_torque=[15,15]
            #for i in range(6, 8):
            #    desire_torque.append(p.getJointState(bodyUniqueId=self.sim,
            #        jointIndex=self.act_joint_indices[i])[3])

            p.setJointMotorControlArray(
                bodyIndex=self.sim,
                jointIndices=paralel_joint,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=desire_velocity,
                forces=desire_torque
                )

        #force_array=[]
        #for i in self.act_joint_indices:
            #force_array.append(p.getJointState(bodyUniqueId=self.sim,
                #jointIndex=i)[3])
        #print(force_array)
        p.stepSimulation()
        time.sleep(0.01)
        if self.testing:
            time.sleep(0.05)

        ob = self.get_obs()
        re_target = np.array([.65,0,.2])
        #re_target = np.concatenate([ob[1][3:], re_target])
        #re_target = np.concatenate([np.array(p.getBasePositionAndOrientation(self.cube)[0]), re_target])
        reward, dist, done = self.cal_reward(ob[1],
                                             re_target,
                                             action)

        self.ep_reward += reward
        self.ep_len += 1
        info = {'reward_so_far': self.ep_reward, 'steps_so_far': self.ep_len,
                'reward': reward, 'step': 1, 'dist': dist}
        return ob, reward, done, info
