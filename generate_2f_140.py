#import python file

    #import generate_all => scalling all part of the robot
import generate_2f_140_all 
    #import geenerator gripper=> scalling the end effector (gripper) only
import generate_2f_140_gripper
    #import generate_finger_l_1=> scalling the fingers lenght only with one ratio scaling
import generate_2f_140_finger_l_1
    #import generate_finger_l_2=> scalling the finger lenght only with two different ratio scaling for each finger link 
import generate_2f_140_finger_l_2
    #import generate_w_gripper=> scalling the end effector width 
import generate_2f_140_w_gripper
    #import generate_w_finger=> scalling the finger width
import generate_2f_140_w_finger

# importing os module 
import os 

#importing shutil
import shutil

#importing random
import random


# change urdf into URDF file function
def change_into_URDF(folder_dir):
    # open urdf file
    Sdirectory="model.urdf"
    Spath = os.path.join(folder_dir, Sdirectory)
    # change urdf into txt file
    Rdirectory="model.txt"
    Rpath=os.path.join(folder_dir,Rdirectory)

    os.rename(Rpath,Spath)

    return Spath
    
# change urdf into txt file function
def change_into_txt(folder_dir):
    # open urdf file
    Sdirectory="model.urdf"
    Spath = os.path.join(folder_dir, Sdirectory)
    # change urdf into txt file
    Rdirectory="model.txt"
    Rpath=os.path.join(folder_dir,Rdirectory)

    os.rename(Spath,Rpath)

    return Rpath



#generate robotic folder function
def cpy_folder(parent_dir,i):
    #source path
    Sdirectory="robotiq_2f_140"
    Spath = os.path.join(parent_dir, Sdirectory)
    #created Path
    save_dir = f'../assets/gen_gripper/2f_140'
    Ddirectory = "robotiq_2f_140_"+str(int(i+1))
    Dpath = os.path.join(save_dir, Ddirectory)

    # Get the list of all files and directories
    # in the root directory
    dir_list = os.listdir(save_dir)

    if Ddirectory in dir_list:
        shutil.rmtree(Dpath)

    #copy folder
    shutil.copytree(src=Spath,dst=Dpath)
    
    return Dpath

#listing the existing Robot
def num_robot_folder():
    
    #existing Path
    save_dir = f'../assets/gen_gripper/2f_140'

    # Get the list of all files and directories
    # in the root directory
    dir_list = os.listdir(save_dir)

    return len(dir_list)

#Delete the existing
def delete_existing():
    #existing Path
    save_dir = f'../assets/gen_gripper/2f_140'

    # Get the list of all files and directories
    # in the root directory
    dir_list = os.listdir(save_dir)

    for d in dir_list:
        Dpath = os.path.join(save_dir, d)
        shutil.rmtree(Dpath)





def generator(choice, dele, number_robot):
    # current directory   
    current_dir = os.getcwd() 
    
    # delete option
    if dele =='n':
        num=num_robot_folder()
    elif dele =='y':
        delete_existing()
        num=0
    else:
        exit()

    for i in range (num,number_robot+num):

        
        # generate multiple folder
        robot_folder=cpy_folder(current_dir,i)
    
        # change urdf into txt file
        txt_dir=change_into_txt(robot_folder)
    
        # start the main algorithm
        if choice==1:
            r=3
            print("ratio :", r) 
            generate_2f_140_all.start_all(txt_dir,r)
        elif choice==2 :
            #r=1
            r=random.gauss(1,0.15)
            print("ratio :", r) 
            generate_2f_140_gripper.start_gripper(txt_dir,r)
        elif choice==3 :
            r=random.gauss(1,0.15)
            print("ratio :", r) 
            generate_2f_140_finger_l_1.start_2f_140_finger(txt_dir,r)
        elif choice==4 :
            r_1=random.gauss(1,0.15)
            print("ratio_1 :", r_1) 
            r_2=random.gauss(1,0.15)
            print("ratio_1 :", r_2) 
            generate_2f_140_finger_l_2.start_2f_140_finger(txt_dir,r_1,r_2)
        elif choice==5 :
            r=random.gauss(1,0.15)
            print("ratio :", r) 
            generate_2f_140_w_gripper.start_2f_140_gripper(txt_dir,r)
        elif choice==6 :    
            r_1=random.gauss(1,0.15)
            
            print("ratio_1 :", r_1) 
            r_2=r_1
            #print("ratio_1 :", r_2) 
            generate_2f_140_w_finger.start_2f_140_finger(txt_dir,r_1,r_2)
        elif choice==7 : 
            r_1=random.gauss(1,0.15)
            print("ratio_1 :", r_1) 
            r_2=random.gauss(1,0.1)
            print("ratio_1 :", r_2) 
            generate_2f_140_w_finger.start_2f_140_finger(txt_dir,r_1,r_2)

        #combination of two algorithm
        elif choice == 8:
            #first, the algorithm scale the gripper 
            r_1=random.gauss(0.8,0.25)
            print("ratio_1 :", r_1) 
            generate_2f_140_gripper.start_gripper(txt_dir,r_1)

            # then, the algoritm scale the finger length
            r_2=random.gauss(0.8,0.25)
            print("ratio_2 :", r_2) 
            r_3=random.gauss(1.,0.)
            print("ratio_3 :", r_3) 
            generate_2f_140_finger_l_2.start_2f_140_finger(txt_dir,r_2,r_3)
        
        #you can create the combination as you desire like in line 163-172 ...
        

   
        # Change TXT into URDF
        change_into_URDF(robot_folder)

if __name__ == "__main__":
    dele = input("Do you wanna delete the existing? (y/n)")
    print("\n")
    choose=input("select mode from 1 to 8:")
    print("\n")
    number_robot=input("How many robot do you want to generate?")
    generator(int(choose), dele,int(number_robot))
