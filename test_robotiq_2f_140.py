import os
import pybullet as p
import pybullet_data
import math



p.connect(p.GUI)
urdfRootPath=pybullet_data.getDataPath()

# Get the list of all files and directories
    # in the root directory

dir_list = os.listdir(urdfRootPath)


dir_list = os.listdir(urdfRootPath)
#print(dir_list)

# robot directory   
robot_dir = f'../assets/gen_gripper/2f_140'

#robotiq 2f_85 folder
robotpath = os.path.join(robot_dir, "robotiq_2f_140_10/model.urdf")

pandaUid = p.loadURDF(robotpath,useFixedBase=True)

#tableUid = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"),basePosition=[0.5,0,-0.65])


planeUid = p.loadURDF(os.path.join(urdfRootPath, "plane.urdf"))

#trayUid = p.loadURDF(os.path.join(urdfRootPath, "tray/traybox.urdf"),basePosition=[0.65,0,0])

p.setGravity(0,0,-10)
#objectUid = p.loadURDF(os.path.join(urdfRootPath, "random_urdfs/000/000.urdf"), basePosition=[0.7,0,0.1])
cubeUid = p.loadURDF(os.path.join(urdfRootPath, "cube_small.urdf"), [.625, 0, 0.0], globalScaling=1)

p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=90, cameraPitch=-40, cameraTargetPosition=[0.55,-0.35,0.2])

state_durations = [1,1,1,1,1]
control_dt = 1./240.
p.setTimestep = control_dt
state_t = 0.
current_state = 0

listjoint=[]

for i in range(p.getNumJoints(pandaUid )):
    listjoint.append(p.getJointInfo(pandaUid,i)[1])
    
print(listjoint)    





while True:
    state_t += control_dt
    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING) 
    if current_state == 0:
        p.setJointMotorControl2(pandaUid, 0, 
                        p.POSITION_CONTROL,-0.197)
        p.setJointMotorControl2(pandaUid, 1, 
                        p.POSITION_CONTROL,-1.17)
        p.setJointMotorControl2(pandaUid, 2, 
                        p.POSITION_CONTROL,math.pi/4)
        p.setJointMotorControl2(pandaUid, 3, 
                        p.POSITION_CONTROL,-2.06)
        p.setJointMotorControl2(pandaUid, 4, 
                        p.POSITION_CONTROL,-1.5715)
        p.setJointMotorControl2(pandaUid, 5, 
                        p.POSITION_CONTROL,1.37)
       
        p.setJointMotorControl2(pandaUid, 8,  
                        p.POSITION_CONTROL,0)
        
        p.setJointMotorControl2(pandaUid, 10,  
                        p.POSITION_CONTROL,0)

        p.setJointMotorControl2(pandaUid, 12,  
                        p.POSITION_CONTROL,0)
        
        p.setJointMotorControl2(pandaUid, 14,  
                        p.POSITION_CONTROL,0)

        p.setJointMotorControl2(pandaUid, 16,  
                        p.POSITION_CONTROL,0)
        p.setJointMotorControl2(pandaUid, 17,  
                        p.POSITION_CONTROL,0)
        
    elif current_state == 1:
       p.setJointMotorControl2(pandaUid, 2,  p.POSITION_CONTROL,1.661)






    elif current_state == 2:

        
        #p.setJointMotorControl2(pandaUid, 2,  p.POSITION_CONTROL,1.661)
        p.setJointMotorControl2(pandaUid, 8,  
                        p.POSITION_CONTROL,-0.65)
        p.setJointMotorControl2(pandaUid, 10,  
                        p.POSITION_CONTROL,0.65)
        #p.setJointMotorControl2(pandaUid, 11,  
                        #p.POSITION_CONTROL,-0.65)
        
    
        p.setJointMotorControl2(pandaUid, 12,  
                        p.POSITION_CONTROL,-0.65)
        p.setJointMotorControl2(pandaUid, 14,  
                        p.POSITION_CONTROL,0.65)

        p.setJointMotorControl2(pandaUid, 16,  
                        p.POSITION_CONTROL,-0.65)
        p.setJointMotorControl2(pandaUid, 17,  
                        p.POSITION_CONTROL,-0.65)
    
        #p.setJointMotorControl2(pandaUid, 13,  
                        #p.POSITION_CONTROL,0.65)
    elif current_state == 3:
        p.setJointMotorControl2(pandaUid, 2,  p.POSITION_CONTROL,math.pi/4)
    
    elif current_state == 4:
        p.setJointMotorControl2(pandaUid, 8,  
                        p.POSITION_CONTROL,0)
        
        p.setJointMotorControl2(pandaUid, 10,  
                        p.POSITION_CONTROL,0)

        p.setJointMotorControl2(pandaUid, 12,  
                        p.POSITION_CONTROL,0)
        
        p.setJointMotorControl2(pandaUid, 14,  
                        p.POSITION_CONTROL,0)

        p.setJointMotorControl2(pandaUid, 16,  
                        p.POSITION_CONTROL,0)
        p.setJointMotorControl2(pandaUid, 17,  
                        p.POSITION_CONTROL,0)
    if state_t >state_durations[current_state]:
        current_state += 1
        if current_state >= len(state_durations):
            current_state = 0
        state_t = 0
           
    p.stepSimulation()
