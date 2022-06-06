# hcp

# 0. Hard_Conditioned_policy
### 0. Activating the environment (Details of the environment can be found in the report)
0.1 conda activate hcp

## Robotiq 2f_140
### 1. Generating Robot (A different number of grippers can be generated for training and testing ) The main folder for this is under :          james/internship/HCP_2f_140 
 1.1 cd generator_robotiq_2f_140
 
 1.2 python generate_2f_140.py

###### Note: for single robot change into "r=1" in code line 129 for example mode 1 scales the whole robot but mode 2 to scale the gripper only, mode 3 is to manipulate the length of the finger. All these can be changed by editing the ratio in the algorithm.


  2.1 cd 2f_140_hcp
  
  2.2.1 python main.py --render --with_kin (if you want to see a simulation)
  
  2.2.2 python main.py --with_kin (if you don't want to see a simulation)
  
### 3. test
  3.1 cd 2f_140_hcp
  
  3.2 python main.py --render --with_kin --test


## Further Work
 1. HCP on 2f_85 can be worked on further as env_base.py is only for 2f_140 lines 140-150 and also in gripper_env.py from line 39

 2. please implement HCP on robotiq_3f. 

 3. If possible TD3 can be explored and implemented instead of DDPG

###### Note: use command line in terminal
