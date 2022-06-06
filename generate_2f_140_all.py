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
    Rdirectory="robotic_2f_140.txt"
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
        
        
        if'scale="' in words and x<3:
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
    listoffingerlinks=[]

    for i in range(len(listofmacs)):
        
        #find link structure
        if ('<link' in listofmacs[i]) :
            magicword.append(listofmacs[i])
        #find the end of link structure
        elif ('</link>' in listofmacs[i]) and len(magicword)!=0 :
            magicword.pop()
        
        #input the content of a particular link structure
        if len(magicword)!=0:
            listoffingerlinks.append(listofmacs[i])

        y=len(listoffingerlinks)-1

        #read the scale of the link origin and scale it
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
    
    listofjoint=[]
    for i in range(len(listofmacs)):
        
        #find every joint structures to relocate
        if '<joint' in listofmacs[i] :
            magicword.append(listofmacs[i])
            #print(magicword)
        #find the end of joint structure
        elif '</joint>' in listofmacs[i] and len(magicword)!=0 :
            magicword.pop()
            #print(magicword)
       
       #input the content of a particular joint structure
        if len(magicword)!=0:
            listofjoint.append(listofmacs[i])

        y=len(listofjoint)-1
        
        #find the coordinate of the joint origin and relocate it
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
    Rdirectory="robotic_2f_140.txt"
    Rpath=os.path.join(folder_dir,Rdirectory)

    os.rename(Spath,Rpath)

    return Rpath

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

def start_all(file_dirictory, ratio):
    # scaling fingers  
        #random scaling for finger 1, finger 2, and middle finger
        #r=random.gauss(1,0.2)

        # scaling 
        rescaling(file_dirictory,ratio )
        # relocate 
        relocate(file_dirictory,ratio )

def generate_2f_140_all():

    # current directory   
    current_dir = os.getcwd() 

    for i in range (5):
    # generate multiple folder
        robot_folder=cpy_folder(current_dir,i)
        
    # change urdf into txt file
        txt_dir=change_into_txt(robot_folder)

    #start the main algorithm
        r=random.gauss(0.75,0.1)
        start_all(txt_dir,r)

    # Change TXT into URDF
        change_into_URDF(robot_folder)



if __name__ == "__main__":
    generate_2f_140_all()
