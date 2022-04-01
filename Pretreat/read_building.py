import numpy as np
import matplotlib.pyplot as plt 
import pandas as pd
import copy 
import itertools
from scipy.linalg import expm, norm
import math
from numpy import cross, eye, dot
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
windows=pd.read_csv('~/Desktop/Drone_code/dbh_win.csv')
off_win=[]
win_h=1.8 
win_below=1.1 
floor=12
floor_h=3
#print(win_set)
scale=round(4.32/0.3048,2)
min_y=min(windows['y'].values)
min_x=min(windows['x'].values)
#print(min_x)
for index,row in windows.iterrows():
    tmp=row
    tmp['x']= round((tmp['x']-min_x)/scale,2)
    tmp['y']= round((tmp['y']-min_y)/scale,2)
    off_win.append(tmp)
max_y=max(off_win[i][1] for i in range(len(off_win)))
name='dbh_win_set.csv'
##here I want to reverse y axis! 
for i in range(len(off_win)):
    off_win[i][1]=round(max_y-off_win[i][1],2)
pd.DataFrame(off_win).to_csv(name,header=True,sep=' ',
                                  index=False,encoding='utf-8')
#changed to meter and minus the offset 
wins=pd.DataFrame(off_win).values.tolist()
win_set=[]
for i in range(int(len(wins)/2)):
    win_set.append(wins[2*i][0:2]+wins[2*i+1][0:2]+[int(wins[2*i][2])])
#print(win_set)
facades=[[]for i in range(7)]
#add vertix z into the locations: 
for i in range(len(win_set)):
    facades[int(win_set[i][4])-1].append(win_set[i]) # categrize windows to each facades
building=pd.read_csv('~/Desktop/Drone_code/dbh_stu.csv',delimiter=' ')
ang_set=building.loc[[0],['angle']].values
num=[];ang=[];clock=[];nom=[];nor=[]
for i in range(len(building)):
    tmp=building.loc[[i],['fac']].values
    num=num+tmp[0][0].split(',')
    tmp=building.loc[[i],['angle']].values
    ang=ang+tmp[0][0].split(',')
    tmp=building.loc[[i],['clock']].values
    clock=clock+tmp[0][0].split(',')
    tmp=building.loc[[i],['nom']].values
    tm=tmp[0][0].split(',')
    for j in range(len(tm)):
        nor.append([int(tm[j].split('/')[k]) for k in range(3)])
#print(nor)
nor_map=dict(zip(num,nor))
#print(nor_map)
ang_map=dict(zip(num,ang))
clock_map=dict(zip(num,clock))
win_z=[[]for i in range(7)]
win_3d=[[]for i in range(7)]
for i in range(len(facades)): 
    for j in range(len(facades[i])):
        ang=ang_map.get(str(facades[i][j][-1]))
        if int(ang)==180 or int(ang)==0: # x is fixed
            facades[i][j]=facades[i][j]+['x']+[int(ang)]
            tmp=[facades[i][j][k] for k in[1,3,4,5,0]]
        else:
            facades[i][j]=facades[i][j]+['y']+[int(ang)]
            tmp=[facades[i][j][k] for k in[0,2,4,5,1]]
        for k in range(floor):
            win_z[i].append([tmp[0],round(win_below+floor_h*k,2),tmp[0],round(win_below+win_h+floor_h*k,2),
                             tmp[1],round(win_below+floor_h*k,2),tmp[1],round(win_below+win_h+floor_h*k,2)]+tmp[2:]+[int(ang),k+1])
            win_3d[i].append([facades[i][j][0],facades[i][j][1],round(win_below+floor_h*k,2),facades[i][j][0],facades[i][j][1],round(win_below+win_h+floor_h*k,2),
                             facades[i][j][2],facades[i][j][3],round(win_below+floor_h*k,2),facades[i][j][2],facades[i][j][3],round(win_below+win_h+floor_h*k,2)]+tmp[2:]+[int(ang),k+1])
# will not be used later 
# for i in range(len(facades)):
#     fname='data/3d/win_3d_'+str(i+1)+'.csv'
#     fac_set=pd.DataFrame(win_3d[i])
#     fac_set.columns=['v_1_x','v_1_y','v_1_z','v_2_x','v_2_y','v_2_z','v_3_x','v_3_y','v_3_z','v_4_x','v_4_y','v_4_z','face','fix_aix','fix_v','angle','floor']
#     fac_set.to_csv(fname,header=True,sep=' ',index=False,encoding='utf-8')
# for i in range(len(facades)):
#     fname='data/win_data_'+str(i+1)+'.csv'
#     fac_set=pd.DataFrame(win_z[i])
#     fac_set.columns=['v_1_x','v_1_y','v_2_x','v_2_y','v_3_x','v_3_y','v_4_x','v_4_y','face','fix_aix','fix_v','angle','floor']
#     fac_set.to_csv(fname,header=True,sep=' ',index=False,encoding='utf-8')
###################################. # get the 3-D and rotation to 2-D coordinates. 
winds=pd.DataFrame(win_set)
winds.columns=['v1_x', 'v1_y','v2_x','v2_y','fac']
pd.DataFrame(winds).to_csv('dbh_win_set2.csv',header=True,sep=' ',
                                  index=False,encoding='utf-8')
faces=copy.deepcopy(facades)
c_faces=copy.deepcopy(facades)
c_clock=[]; #1,2,..7 
clock=[] # 7,6,5..1 
for i in range(len(faces)):
    if(faces[i][0][-2])=='x': #order by y 
        if (clock_map.get(str(i+1))=='1'):  # y +increase 
            faces[i].sort(key=lambda x:x[1], reverse=False)
        else:
            faces[i].sort(key=lambda x:x[1], reverse=True)
    else:
        if (clock_map.get(str(i+1))=='1'):
            faces[i].sort(key=lambda x:x[0], reverse=False)
        else:
            faces[i].sort(key=lambda x:x[0], reverse=True) 
all_fac=[]
for i in faces:
    all_fac=all_fac+i
clock=pd.DataFrame(all_fac)
clock.columns=['v_1_x','v_1_y','v_2_x','v_2_y','face','fix_aix','angle']
clock.insert(0,'id',clock.index,True)
clock.to_csv('~/Desktop/Drone_code/data/layer_win.csv',header=True,sep=' ',
                                  index=False,encoding='utf-8')
#print(clock)
##### get all windows for 12 layers: 
layers=[[]for i in range(floor)]
all_floors=[]
num=0
for i in range(floor):
    for j in range(len(clock)):  
        all_floors.append([num]+all_fac[j][0:2]+[round(win_below+floor_h*i,2)]+all_fac[j][0:2]+[round(win_below+win_h+floor_h*i,2)]+all_fac[j][2:4]
                     +[round(win_below+win_h+floor_h*i,2)]+all_fac[j][2:4]
                     +[round(win_below+floor_h*i,2)]+all_fac[j][4:]+[i+1])
        num=num+1
num=0
for i in range(floor):
    for j in range(len(clock)):
        layers[i].append(all_floors[num])
        num=num+1
layer0=pd.DataFrame(layers[2])
al_ly_win=pd.DataFrame(all_floors)
al_ly_win.columns=['id','v_1_x','v_1_y','v_1_z','v_2_x','v_2_y','v_2_z','v_3_x','v_3_y','v_3_z','v_4_x','v_4_y','v_4_z','face','fix_aix','ang','layer']
al_ly_win.to_csv('~/Desktop/Drone_code/data/all_win.csv',header=True,sep=' ',
                                  index=False,encoding='utf-8')
print(al_ly_win)
D=np.zeros((len(all_fac),len(all_fac)))  # get the distance among the same layer. 
max_v=len(all_fac)-1 
##### here did not be used.
for i in range(len(all_fac)):    # get the 
    for j in range(i, len(all_fac)):
        d=min(abs(j-i),max_v-j+i+1)
        D[i][j]=d
        D[j][i]=d
print(D)
################.  # after rotation 
dic_ly=al_ly_win.groupby('face').groups
#Here we do something for rotation: get each windows in each facade, id, two vertices, and get the list version 
fa_ro=[[]for i in range(len(faces))]
st_nor=np.array([0,-1,0])
for i in range(len(faces)):
    a=np.array(nor_map.get(str(i+1)))
    tmp=[all_floors[j] for j in list(dic_ly.get(i+1))]
    if str(np.cross(a,st_nor))=='[0 0 0]':
        #print("here it is invaliable",a[1])
        for k in range(len(tmp)):
            fa_ro[i].append([tmp[k][0]]+tmp[k][1:4]+tmp[k][10:13]+[a[1]]+[tmp[k][-1]])
       
    else:
        for k in range(len(tmp)):
            v1=rot(a,np.array([0,-1,0]),np.array(tmp[k][1:4]))
            v2=rot(a,np.array([0,-1,0]),np.array(tmp[k][10:13]))
            fa_ro[i].append([tmp[k][0]]+list(v1)+list(v2)+[-1]+[tmp[k][-1]])
    #print(fa_ro[i])
ro_win=[[]for i in range(len(faces))]
for i in range(len(faces)): # get the 2d coordinates for windows in each facade 
    for j in range(len(fa_ro[i])):
        ro_win[i].append([fa_ro[i][j][k] for k in [0,1,3,1]]+[round(fa_ro[i][j][3]+win_h,2)]+
                         [fa_ro[i][j][4],round(fa_ro[i][j][3]+win_h,2)]+[fa_ro[i][j][k] for k in [4,3]]+[i+1,'y']+[fa_ro[i][j][k] for k in [2,-1,-2]])
    fname='data/win_ro_f_'+str(i+1)+'.csv'
    fac_set=pd.DataFrame(ro_win[i])
    fac_set.columns=['id','v_1_x','v_1_y','v_2_x','v_2_y','v_3_x','v_3_y','v_4_x','v_4_y','face','fix_aix','fix_v','floor','dir']
    fac_set.to_csv(fname,header=True,sep=' ',index=False,encoding='utf-8')  
####################################### Try to get the Fov coverage. 


















