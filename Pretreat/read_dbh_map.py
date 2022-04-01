import numpy as np
import matplotlib.pyplot as plt 
import pandas as pd
import copy 
import itertools
from scipy.linalg import expm, norm
import math
import re
from numpy import cross, eye, dot
import sys
def M(axis, theta):
    return expm(cross(eye(3), axis/norm(axis)*theta))
def rot(a,b,v):  # a to b 
    Lx=np.sqrt(a.dot(a))
    Ly=np.sqrt(b.dot(b))
    theta = math.acos(np.dot(a,b)/(Lx*Ly))  # here theta always < 180, from a to b, a is the facade, b is our standarded axis
    axis=np.cross(a,b)  # here should be a x b. so a to b < 180, up, the same. 
    return np.round(np.dot(M(np.cross(a,b), theta), v),2) 
def rot_bac(a,b,v):
    Lx=np.sqrt(a.dot(a))
    Ly=np.sqrt(b.dot(b))
    theta = math.acos(np.dot(a,b)/(Lx*Ly))  
# map=pd.read_csv('../data/dbh_fire.fds')
#f=open('/Users/fangqiliu/Desktop/fire_simulation/dbh_fire/dbh_to_door.fds','r')
f=open('../data/dbh_with_fire_door.fds','r')
#print(sys.path)
# f2=open('/Users/fangqiliu/Desktop/fire_simulation/dbh_fire/dbh_no_door.fds','w+')
# f3=open('/Users/fangqiliu/Desktop/fire_simulation/dbh_fire/dbh_no_face.fds','w+')
#f=open('../data/all_win.csv')
#f2.write('dbh_door.fds\n')
# for line in (f.readlines())[1:]:
# #     if line=='dbh_fire.fds':
# #         pass
#     k=line.split(', ')
#     if re.match("&OBST ID='AcDbPolyFaceMesh*",str(k[0]))==None and re.match("&OBST ID='AcDbBlockReference*",str(k[0]))==None:
#         f2.write(line)
#     else:
#         co=k[1].split(',')
#         co[0]=co[0].split('=')[1]
#         if round(float(co[-1])-float(co[-2]),2)!=2.01:
#             #f2.write(re.sub(r'AcDbPolyFaceMesh||AcDbBlockReference','Door',line,count=0,flags=0))
#             # print(line)
#             f2.write(line)
#         else:
#            # print(line)
#             f2.write(re.sub(r'AcDbPolyFaceMesh|AcDbBlockReference','Door',line,count=0,flags=0))
#f3.write('dbh_face.fds\n')

# for line in (f.readlines()[1:]):
#     print(line)
#     k=line.split(', ')
#     if re.match("&OBST ID='AcDbPolyFaceMesh*",str(k[0]))!=None or re.match("&OBST ID='AcDbBlockReference*",str(k[0]))!=None:
#         print("why",line)
    
f=open('../data/dbh_dev_fire_door.fds','r')

to_file="../data/dbh_no_fire_door.fds"
def put_dev_fire(f,to_file=None,sources=None):
    fire_dic={}
    title=f"{to_file}_{sources[:]}.fds"
    f2=open(title,'w+')
    print(title)
    seq=0
    before=[]
    after=[]
    content=f.readlines()
    f.close()
    for line in content:
        k=line.split(', ')
        if re.match("&OBST ID='fire_3 *",str(k[0]))!=None:
            fire_dic[k[0]]=line
    fire_seed=list(fire_dic.keys())
    fire_sources=[fire_seed[i] for i in sources]
    #print(fire_sources)
    for line in content:
        k=line.split(', ')
        if re.match("&OBST ID='fire_*",str(k[0]))==None or k[0] in fire_sources:
            f2.write(line)
    f2.close()        

def put_smoke(f,to_file=None):
    dev_dic={}
    seq=0
    content=f.readlines()
    f.close()
    for line in content:
        k=line.split(', ')
        if re.match("&DEVC ID='HD_ *",str(k[0]))!=None:
            #dev_dic[k[0]]=k[-1]
            name=k[0].split("'")[1][2:]
            sd_name=f"&DEVC ID='SD{name}', PROP_ID='Cleary Photoelectric P1', {k[-1]}"
            dev_dic[k[0]]=sd_name
           # print(sd_name)
    print(dev_dic)
    for line in content:
        k=line.split(', ')
        if re.match("&DEVC ID='HD*",str(k[0]))!=None:
            print(line)
            print(dev_dic.get(k[0]))
#             f2.write(line)
#     f2.close()      
def put_no_door_fire(f,to_file=None,sources=None):   # no dev
    fire_dic={}
    title=f"{to_file}_{sources[:]}.fds"
    f2=open(title,'w+')
    print(title)
    seq=0
    before=[]
    after=[]
    content=f.readlines()
    f.close()
    for line in content:
        k=line.split(', ')
        if re.match("&OBST ID='fire_3 *",str(k[0]))!=None:
            fire_dic[k[0]]=line
    fire_seed=list(fire_dic.keys())
    print(fire_seed)
    fire_sources=[fire_seed[i] for i in sources]
    for line in content:
        k=line.split(', ')
        if re.match("&OBST ID='fire_*",str(k[0]))==None or k[0] in fire_sources:
            if re.match("&DEVC ID=*",str(k[0]))==None and re.match("&OBST ID='Door -",str(k[0]))==None: 
               # print(line)
                f2.write(line) 
    f2.close()    
def put_no_door_no_window_fire(f,to_file=None,sources=None):   # no dev
    fire_dic={}
    title=f"{to_file}_{sources[:]}.fds"
    f2=open(title,'w+')
    print(title)
    seq=0
    before=[]
    after=[]
    content=f.readlines()
    f.close()
    for line in content:
        k=line.split(', ')
        if re.match("&OBST ID='fire_3 *",str(k[0]))!=None:
            fire_dic[k[0]]=line
    fire_seed=list(fire_dic.keys())
    print(fire_seed)
    fire_sources=[fire_seed[i] for i in sources]
    for line in content:
        k=line.split(', ')
        if re.match("&OBST ID='fire_*",str(k[0]))==None or k[0] in fire_sources:
            if re.match("&DEVC ID=*",str(k[0]))==None and re.match("&OBST ID='Door -",str(k[0]))==None: 
                if re.match("&HOLE ID='w*",str(k[0]))==None:
                    #print(line)
                    f2.write(line)
    f2.close()   
def put_fire(f,to_file,sources=None):   # no dev
    fire_dic={}
    title=f"{to_file}_{sources[:]}.fds"
#     title=f"{to_file}.fds"
#     print(title)
    print(to_file)
    f2=open(title,'w+')
    content=f.readlines()
    f.close()
    for line in content:
        k=line.split(', ')
        if re.match("&OBST ID='fire_3 *",str(k[0]))!=None:
            fire_dic[k[0]]=line
        if re.match("&SURF ID='fire'",str(k[0]))!=None:
            print(line)

    fire_seed=list(fire_dic.keys())
    fire_sources=[fire_seed[i] for i in sources]
#     content2=f.readlines()
#     for i in content2[:-1]:
#         f2.write(i)
#     for i in fire_seed:
#         f2.write(fire_dic.get(i))
  #  f2.write(content2[-1])
#     fire_sources=fire_seed
    for line in content:
        k=line.split(', ')
        if re.match("&OBST ID='fire_*",str(k[0]))==None or k[0] in fire_sources:
            #if re.match("&DEVC ID=*",str(k[0]))==None: 
            f2.write(line)
    f2.close()        
    
                 
f=open('../data/dbh_fire_door.fds','r')
#to_file="../data/dbh_no_dev_fire_door"           
# to_file="../data/dbh_no_door_dev_fire_door"
# to_file="../data/dbh_no_door_no_window_dev_fire_door"
#put_no_door_fire(f,to_file,[1,3])

#put_no_door_no_window_fire(f,to_file,[1,3])
to_file="../dbh/dbh_fire_"
#f1=open('../dbh/dbh_better_dev.fds','r')
f=open('../dbh/dbh_fire_better.fds','r')
put_fire(f,to_file,[1,3])





#to_file="../data/dbh_dev_fire_door"     
# f=open("../data/dbh_smoke_fire_door.fds",'r')    
# put_smoke(f,to_file)
