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



# scaling algorithm
##replace_scaling function

def scale_replace(sentence, r):
    words = []
    number=[]
    x=0

    for word in sentence.split():
        
        
        if'scale="'  in words and x<3:
            x=x+1
            if x==1:
                number.append(float(word))
                words.append(str(float(word)*r))
                #print(word)
            else:
                number.append(float(word))
                words.append(str(float(word)*1))
        elif 'size="' in words and x<3:
            x=x+1
            if x==1:
                number.append(float(word))
                words.append(str(float(word)*r))
                #print(word)
            else:
                number.append(float(word))
                words.append(str(float(word)*1))
        else:
            words.append(word)
            #print(word)

    return ' '.join(words)

# search the scaling function in the file
def rescaling(Rpath,r, finger):

    listofmacs=[]
    with open(Rpath,"r") as reader:
        for line in reader.readlines():
            listofmacs.append(line)

    magicword=[]
    listoffingerlinks=[]

    for i in range(len(listofmacs)):
        
        #find a link structure
        if ('<link' in listofmacs[i] and finger in listofmacs[i]):
            magicword.append(listofmacs[i])
        #end of a link structure    
        elif ('</link>' in listofmacs[i]) and len(magicword)!=0 :
            magicword.pop()
        
        
        
        #gather every information in a link structure
        if len(magicword)!=0:
            listoffingerlinks.append(listofmacs[i])

        y=len(listoffingerlinks)-1

        #find a scaling function and rescale the link
        if len(listoffingerlinks)!=0 and 'mesh' in listoffingerlinks[y]:
            #print(listoffingerlinks[y])
            listofmacs[i]=scale_replace(listofmacs[i],r)
        elif len(listoffingerlinks)!=0 and 'box' in listoffingerlinks[y]:
            #print(listoffingerlinks[y])
            listofmacs[i]=scale_replace(listofmacs[i],r)

    with open(Rpath,"w") as f:
        for j in listofmacs:
            f.write(j)
            #f.write("\n")

#relocate algorithm 

def replace(sentence,r):
    words = []
    number=[]
    x=0
    for word in sentence.split():
        
        
        if'xyz="'  in words and x<3:
            x=x+1
            if x==2:
                number.append(float(word))
                words.append(str(float(word)*r))
                #print(word)
            else:
                number.append(float(word))
                words.append(str(float(word)*1))
            
        else:
            words.append(word)

            

    return ' '.join(words)




def relocate(Rpath,r,joint):
        
     
    listofmacs=[]
    with open(Rpath,"r") as reader:
        for line in reader.readlines():
            listofmacs.append(line)

    magicword=[]
    
    listofjoint=[]
    for i in range(len(listofmacs)):
        #find joint structure
        if '<joint' in listofmacs[i] and joint in listofmacs[i]:
            magicword.append(listofmacs[i])
            #print(magicword)
        #end of joint structure
        elif '</joint>' in listofmacs[i] and len(magicword)!=0 :
            magicword.pop()
            #print(magicword)

        
            
        #gather every information in a joint structure
        if len(magicword)!=0:
            listofjoint.append(listofmacs[i])

        y=len(listofjoint)-1
        
        #gather information about the origin coordinate and relocate it        
        if len(listofjoint)!=0 and 'origin' in listofjoint[y]:
            #print(listofjoint[y])
            listofmacs[i]=replace(listofmacs[i],r)

    
    
   
    with open(Rpath,"w") as f:
        for j in listofmacs:
            f.write(j)
            


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

def start_2f_140_finger(txt_dir, ratio_1, ratio_2):

    # scaling fingers  
        
        #ratio_1 for inner_knuckle and outer_finger
        #ratio_2 for "inner_finger_pad"
        

    # scaling inner_knuckle_joint
    rescaling(txt_dir,ratio_1,"inner_knuckle")

    # scaling outer_finger
    rescaling(txt_dir,ratio_1,"outer_finger")

    # scaling outer_finger
    rescaling(txt_dir,ratio_1,"inner_finger")

    # scaling inner_finger_pad
    rescaling(txt_dir,ratio_2,"inner_finger_pad")

   





def generate_2f_140_finger():

    # current directory   
    current_dir = os.getcwd() 

    # Print the current working  
    # directory (CWD) 
    #print("Current working directory:", current_dir ) 
    
    for i in range (1):
    # generate multiple folder
        robot_folder=cpy_folder(current_dir,i)
        


    # change urdf into txt file
        txt_dir=change_into_txt(robot_folder)
        

    #start the  main algorithm
        r=random.gauss(1,0.1)
        start_2f_140_finger(txt_dir,r)

    # Change TXT into URDF
        change_into_URDF(robot_folder)


if __name__ == "__main__":
    generate_2f_140_finger()
