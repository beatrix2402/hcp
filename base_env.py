import json
import os
from typing import Sized

import random
import numpy as np
import gym
from gym import spaces
from numpy.random import f
import pybullet as p
from urdfpy import URDF
from pybullet_utils import bullet_client
from util import rotations
import pybullet_data
from collections import OrderedDict


class BaseEnv:

    def __init__(self,
                 robot_folders,
                 robot_dir,
                 render,
                 tol=0.02,
                 train=True,
                 with_kin=None,
                 ):
        self.with_kin = with_kin
        self.goal_dim = 6


        self.reward_range = (-np.inf, np.inf)
        self.spec = None
        self.dist_tol = tol
        self.pc = bullet_client.BulletClient(p.GUI if render else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.ll = [-3.14, -3.14, -3.14, -3.14, -3.14, -3.14]
        self.ul = [3.14, 3.14, 3.14, 3.14, 3.14, 3.14]
        self.end_factor = 7
        self.finger_height_offset = 0
        self.robots = []
        for folder in robot_folders:
            self.robots.append(os.path.join(robot_dir, folder))

        self.dir2id = {folder: idx for idx, folder in enumerate(self.robots)}
        self.robot_num = len(self.robots)
        
        if train:
            self.test_robot_num = min(10, self.robot_num) #change the min
            self.train_robot_num = self.robot_num - self.test_robot_num
            self.test_robot_ids = list(range(self.train_robot_num,
                                             self.robot_num))
            self.train_test_robot_num = min(10, self.train_robot_num) #change the min
            self.train_test_robot_ids = list(range(self.train_test_robot_num))
            self.train_test_conditions = self.train_test_robot_num
            self.testing = False
        else:
            self.test_robot_num = self.robot_num
            self.train_robot_num = self.robot_num - self.test_robot_num
            self.test_robot_ids = list(range(self.robot_num))
            self.testing = True

        self.test_conditions = self.test_robot_num

        print('Train robots: ', self.train_robot_num)
        print('Test robots: ', self.test_robot_num)
        self.reset_robot(0)

        self.ob_dim = self.get_obs()[0].size
        print('Ob dim: ', self.ob_dim)

        high = np.inf * np.ones(self.ob_dim)
        low = -high
        self.observation_space = spaces.Box(low, high, dtype=np.float32)

        self.ep_reward = 0
        self.ep_len = 0

    def reset(self, robot_id=None):
        raise NotImplementedError

    def step(self, action):
        raise NotImplementedError
    
    def reset_robot_pose(self, robot_id, act_joint_indices):
        print(act_joint_indices)


        p.resetDebugVisualizerCamera(cameraDistance=0.35, cameraYaw=0, cameraPitch=-25, cameraTargetPosition=[0.55,-0.35,0.2])
        random_location=random.gauss(1.55,0.001)
        desired_joint_positions = [-0.197, -1.8, random_location, -2.06, -1.5715, 1.37]
        p.setJointMotorControlArray(
            bodyIndex=robot_id,
            jointIndices=act_joint_indices[:6],
            controlMode=p.POSITION_CONTROL,
            targetPositions=desired_joint_positions
            #forces=torque,
        )
        for i in range(10):
            p.stepSimulation()
        desired_joint_positions = p.calculateInverseKinematics(
                                            self.sim, self.end_factor,
                                            [.65, 0, .2-self.finger_height_offset],[1,1,0,0],
                                            lowerLimits=self.ll, upperLimits=self.ul,  residualThreshold=1e-5)[:6]
        p.setJointMotorControlArray(
            bodyIndex=robot_id,
            jointIndices=act_joint_indices[:6],
            controlMode=p.POSITION_CONTROL,
            targetPositions=desired_joint_positions
            #forces=torque,
        )
        for i in range(10):
            p.stepSimulation()
        return desired_joint_positions

    
    def update_action_space(self):
        self.ctrl_high = np.ones(5) * .05
        self.ctrl_low = -self.ctrl_high
        self.action_space = spaces.Box(self.ctrl_low, self.ctrl_high, dtype=np.float32)


    def scale_action(self, action):
        act_k = (self.action_space.high - self.action_space.low)/2.
        act_b = (self.action_space.high + self.action_space.low)/2.
        #print(act_k * action + act_b)
        return act_k * action + act_b
        #return [.01 if b>0 else -.01 for b in action]

    def reset_robot(self, robot_id):
        
        self.robot_folder_id = self.dir2id[self.robots[robot_id]]
        robot_file = os.path.join(self.robots[robot_id], 'model.urdf')
        p.resetSimulation()
        p.setGravity(0,0,-9.81)
        robot_pose = [0,0,0]
        cube_pose = [.65, 0, 0.025]        
        self.sim = p.loadURDF(robot_file, basePosition=robot_pose,
                              useFixedBase=1, physicsClientId=self.pc._client)
        self.sim_urdf = URDF.load(robot_file)
        self.robot_name = self.sim_urdf.name
        
        if 'robotiq_3f' in self.robot_name:
            self.act_joint_indices = [0,1,2,3,4,5, 9, 10,14,18]

        elif 'robotiq_2f_140' in self.robot_name:
            self.act_joint_indices = [0,1,2,3,4,5, 8,12, 10,14]

        #pasd = []
        #for i in range(p.getNumJoints(self.sim)):
            #pasd.append((p.getJointInfo(self.sim,i)[0],p.getJointInfo(self.sim,i)[1]))
        #print(pasd)
        self.reset_robot_pose(self.sim, self.act_joint_indices)
        self.plane = p.loadURDF("plane.urdf")
        #self.tray = p.loadURDF('tray/tray.urdf', [.7, 0, 0],[0,0,1,1],useFixedBase=True,)

        self.cube = p.loadURDF('cube_small.urdf', cube_pose, globalScaling=random.gauss(1.8,0.05))
        #if self.with_kin:
            

        for i in self.act_joint_indices[6:]:
            p.changeDynamics(robot_id, i, lateralFriction=5)
        self.update_action_space()

    def test_reset(self, cond):
        robot_id = self.test_robot_ids[cond]
        return self.reset(robot_id=robot_id)

    def train_test_reset(self, cond):
        robot_id = self.train_test_robot_ids[cond]
        return self.reset(robot_id=robot_id)

    def cal_reward(self, s, goal, a ):
        
        reached = np.linalg.norm(s[:3] - goal[:3])
        dist = np.linalg.norm(s[3:] - goal[:3])

        flooring = len(p.getContactPoints(self.sim, self.plane))
        contact_pts = len(p.getContactPoints(self.sim, self.cube))
        link_set = set({})
        if  contact_pts!= 0:
            for i in range(contact_pts):
                link_set.add(p.getContactPoints(self.sim, self.cube)[i][3])

        
        if dist < self.dist_tol:
            done = True
            reward_dist = 10
        elif flooring != 0:
            done = False
            reward_dist = -10
        elif dist < .1:
            done = False
            reward_dist = 5-dist

        elif contact_pts!=0:
            done = False
            
            if 'robotiq_3f' in self.robot_name:
                if (8 in link_set) or (11 in link_set) or (15 in link_set) or (19 in link_set):
                    reward_dist = 1.25-dist
                elif (12 in link_set) and (16 in link_set) and (20 in link_set):
                    reward_dist = 1-dist
            
                elif (12 in link_set) or (16 in link_set) or (20 in link_set):
                    reward_dist = .5-dist
                else:
                    reward_dist = .25-dist
            elif 'robotiq_2f_140' in self.robot_name:
                
                
            
                if (11 in link_set) or (14 in link_set) or (15 in link_set):
                    reward_dist = 1-dist
                else:
                    reward_dist = .5-dist

        elif np.linalg.norm(s[:2] - goal[:2]) < .02:
            done = False
            
            reward_dist = -(s[3]-goal[2])-dist
        else:
            done = False
            reward_dist = -reached -dist
        reward = reward_dist
        final_dist = [reached,dist]

        

        return reward, final_dist , done

        

    def get_obs(self):

        qpos = self.get_qpos(self.sim)
        qvel = self.get_qvel(self.sim)

        ob = np.concatenate([qpos, qvel])

        endfactor_pos  = np.array((p.getLinkState(self.sim, self.end_factor)[4]))

        # find  gripper center of mass 
        #if 'robotiq_3f' in self.robot_name:
            
    
        #    palmlinkcoordinate=[]
        #    sumcoordinates=np.array([0,0,0])
        #    b=0
        #    for j in range (self.end_factor,21):
        
        #        coordinates=np.array((p.getLinkState(self.sim, j)[0]))
        #        sumcoordinates=sumcoordinates+coordinates
        #        palmlinkcoordinate.append(coordinates)
                #print("coordinate of",j,"link is",palmlinkcoordinate[b])
        #        b=b+1
    
            #print("end_rep1",end_pos)
        #    centalmass=sumcoordinates/len(palmlinkcoordinate)



        #endfactor_pos = centalmass - np.array([0,0,self.finger_height_offset])
        #joint_states = p.getJointStates(self.sim, self.act_joint_indices[6:])
        #g = [j[0] for j in joint_states]
        #print("lenght g is",len(g))
        
        #if 'robotiq_3f' in self.robot_name:
        #    gripper_qpos = np.array([g[0], g[1], g[2], g[3], g[4], g[5],g[6],g[7],g[8],g[9],g[10]])
        

        
        
        if self.with_kin:
            link_rela =self.get_pos_rot(self.sim)
            ob = np.concatenate([ob, link_rela])
            #print("first ob", ob)
        
        target_pos = np.array([.65, 0,.2]).flatten()
        ob = np.concatenate([ob, target_pos])
        
        

        achieved_goal = np.array(p.getBasePositionAndOrientation(self.cube)[0])
        
        ref_point = np.concatenate([endfactor_pos,achieved_goal])

        #ref_point = np.concatenate([endfactor_pos,ref_point])
        return ob, ref_point

    def get_qpos(self, sim):
        angle_noise_range = 0.02
        #attained every information of act_joint
        joint_states = p.getJointStates(self.sim, self.act_joint_indices[0:])
        #attained angle_position of act_joint
        qpos = np.array([j[0] for j in joint_states])
         
        qpos += np.random.uniform(-angle_noise_range, angle_noise_range, len(qpos))
        #qpos = np.pad(qpos, (0, 7 - self.act_dim), mode='constant', constant_values=0)
        return qpos.reshape(-1)
    
    def get_qvel(self, sim):
        velocity_noise_range = 0.02
        #attained every information of act_joint
        joint_states = p.getJointStates(self.sim, self.act_joint_indices[0:])
        #attained angle_velocity of act_joint
        qvel = np.array([j[1] for j in joint_states])

        qvel += np.random.uniform(-velocity_noise_range, velocity_noise_range, len(qvel))
        #qvel = np.pad(qvel, (0, 7 - self.act_dim), mode='constant', constant_values=0)
        return qvel.reshape(-1)

    def get_pos_rot(self,sim):
        #define x_rot and x_rot list as an output of this algorithm
        x_pos=[]
        x_rot=[]

        #get base link of the robot oriention and position
        base_world_quarternion= p.getBasePositionAndOrientation(self.sim)[1]
        base_world_matrix=p.getMatrixFromQuaternion(base_world_quarternion)
        
        base_world_position= p.getBasePositionAndOrientation(self.sim)[0]

        #define world position and orientation matrix for each links
        world_position=[base_world_position]
        world_orientation=[base_world_matrix]
        
        # world position and orientation matrix from each links
        for i in range(p.getNumJoints(self.sim)): #note that p.getNumJoints(self.sim) equals to number of existing links
            world_position.append(p.getLinkState(self.sim,i)[4])
            
            world_quarternion=p.getLinkState(self.sim,i)[5]
            world_matrix=p.getMatrixFromQuaternion(world_quarternion)
            world_orientation.append(world_matrix)

        # get relative position
        for j in range(len(world_position)-1):
            pos1=np.array(world_position[j])
            pos2=np.array(world_position[j+1])
            relative_pos=pos2-pos1
            x_pos.append(relative_pos)

        # get relative rotation
        for k in range(len(world_orientation)-1):
            #reshape into 3 by 3 np array data
            mat1=np.array(world_orientation[k]).reshape((3,3))
            mat2=np.array(world_orientation[k+1]).reshape((3,3))
            rela_mat=np.dot(np.linalg.inv(mat1),mat2)
            x_rot.append(rotations.mat2euler(rela_mat))
        
        xpos=np.array(x_pos).flatten()
        xrot=np.array(x_rot).flatten()

        #xpos = np.pad(xpos, (0, (7 - self.act_dim) * 3), mode='constant', constant_values=0)
        #xrot = np.pad(xrot, (0, (7 - self.act_dim) * 3), mode='constant', constant_values=0)
        
        ref_pt_xpos = np.array(p.getBasePositionAndOrientation(self.cube)[0])
        
        relative_pos = ref_pt_xpos - pos2

        ref_pt_xmat = np.array(p.getMatrixFromQuaternion(p.getBasePositionAndOrientation(self.cube)[1])).reshape(3,3)

        rot_euler = rotations.mat2euler(np.dot(np.linalg.inv(mat2), ref_pt_xmat))
        
        xpos = np.concatenate((xpos, relative_pos.flatten()))
        xrot = np.concatenate((xrot, rot_euler.flatten()))
        
        
        pos_rot=np.concatenate((xpos,xrot))

        return pos_rot


            

    def close(self):
        pass
