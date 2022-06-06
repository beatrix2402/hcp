# importing os module 
import os 

#importing shutil
import shutil

#importing random
import random

# change urdf into URDF file function
def change_into_URDF(folder_dir):
    # open urdf file
    Sdirectory="robotiq_2f_140.urdf"
    Spath = os.path.join(folder_dir, Sdirectory)
    # change urdf into txt file
    Rdirectory="robotiq_2f_140.txt"
    Rpath=os.path.join(folder_dir,Rdirectory)

    os.rename(Rpath,Spath)

    return Spath



# scaling algorithm
##replace_scaling function

def scale_replace(sentence, r):
    words = []
    number=[]
    x=0

    for word in sentence.split():
        
        
        if'scale="'  in words and x<3:
            x=x+1
            number.append(float(word))
            words.append(str(float(word)*r))
            #print(word)
        elif 'size="' in words and x<3:
            x=x+1
            number.append(float(word))
            words.append(str(float(word)*r))
        else:
            words.append(word)
            #print(word)

    return ' '.join(words)

# search the scaling function in the file
def rescaling(Rpath,r):

    listofmacs=[]
    with open(Rpath,"r") as reader:
        for line in reader.readlines():
            listofmacs.append(line)

    magicword=[]
    start=[]
    listoffingerlinks=[]

    for i in range(len(listofmacs)):
        #print(listofmacs[i])
        if '<!--palm-->' in listofmacs[i]:
            start.append(listofmacs[i])
        if len(start)!=0:
            if ('<link' in listofmacs[i]) :
                magicword.append(listofmacs[i])
            elif ('</link>' in listofmacs[i]) and len(magicword)!=0 :
                magicword.pop()
            
            
            
            if len(magicword)!=0:
                listoffingerlinks.append(listofmacs[i])

            y=len(listoffingerlinks)-1


            if len(listoffingerlinks)!=0 and 'mesh' in listoffingerlinks[y]:
                #print(listoffingerlinks[y])
                listofmacs[i]=scale_replace(listofmacs[i],r)
            elif len(listoffingerlinks)!=0 and 'box' in listoffingerlinks[y]:
                #print(listoffingerlinks[y])
                listofmacs[i]=scale_replace(listofmacs[i],r)
    with open(Rpath,"w") as f:
        for j in listofmacs:
            f.write(j)
            f.write("\n")
    #print(listoffingerlinks)
    #print("\n")

#relocate algorithm 

def replace(sentence,r):
    words = []
    number=[]
    x=0
    for word in sentence.split():
        
        
        if'xyz="'  in words and x<3:
            x=x+1
            number.append(float(word))
            words.append(str(float(word)*r))
            
        else:
            words.append(word)

            

    return ' '.join(words)




def relocate(Rpath,r):
     
    listofmacs=[]
    with open(Rpath,"r") as reader:
        for line in reader.readlines():
            listofmacs.append(line)

    magicword=[]
    start=[]
    listofjoint=[]

    for i in range(len(listofmacs)):
        #print(listofmacs[i])
        #print(type(listofmacs[i]))
        if '<!--palm-->' in listofmacs[i]:
            start.append(listofmacs[i])
        if len(start)!=0:
            if '<joint' in listofmacs[i] :
                magicword.append(listofmacs[i])
                #print(magicword)
            elif '</joint>' in listofmacs[i] and len(magicword)!=0 :
                magicword.pop()
                #print(magicword)

            
                
            
            if len(magicword)!=0:
                listofjoint.append(listofmacs[i])


            y=len(listofjoint)-1
            
            
            
            if len(listofjoint)!=0 and 'origin' in listofjoint[y]:
                #print(listofjoint[y])
                listofmacs[i]=replace(listofmacs[i],r)

    with open(Rpath,"w") as f:
        for j in listofmacs:
            f.write(j)
            
# change urdf into txt file function
def change_into_txt(folder_dir):
    # open urdf file
    Sdirectory="robotiq_2f_140.urdf"
    Spath = os.path.join(folder_dir, Sdirectory)
    # change urdf into txt file
    Rdirectory="robotiq_2f_140.txt"
    Rpath=os.path.join(folder_dir,Rdirectory)

    os.rename(Spath,Rpath)

    return Rpath

#start the main algorithm
def  start_gripper(file_directory,ratio):
    # scaling fingers  
        #random scaling for finger 1, finger 2, and middle finger
    #r=random.gauss(1,0.2)
    #r=1.25
    
    if ratio >0.8:
            # scaling 
        rescaling(file_directory,ratio )
            # relocate 
        relocate(file_directory,ratio )
    else:
        # scaling 
        rescaling(file_directory,ratio )
        # relocate 
        relocate(file_directory,ratio  )
        



#generate robotic folder function
def cpy_folder(parent_dir,i):
    #source path
    Sdirectory="robotiq_2f_140"
    Spath = os.path.join(parent_dir, Sdirectory)
    #created Path
    Ddirectory = "robotiq_2f_140_"+str(int(i+1))
    Dpath = os.path.join(parent_dir, Ddirectory)

    # Get the list of all files and directories
    # in the root directory
    dir_list = os.listdir(parent_dir)

    if Ddirectory in dir_list:
        shutil.rmtree(Dpath)

    #copy folder
    shutil.copytree(src=Spath,dst=Dpath)
    
    return Dpath

def generate_2f_140_grippper():
    # current directory   
    current_dir = os.getcwd() 

    for i in range (1):
        # generate multiple folder
        robot_folder=cpy_folder(current_dir,i)
    
        # change urdf into txt file
        txt_dir=change_into_txt(robot_folder)
    
        # start the main algorithm
        r=random.gauss(1,0.2)
        start_gripper(txt_dir, r)

   
        # Change TXT into URDF
        change_into_URDF(robot_folder)


if __name__ == "__main__":
    generate_2f_140_grippper()
