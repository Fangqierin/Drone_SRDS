
import numpy as np
import matplotlib.pyplot as plt
import sys
import geopandas as gpd
sys.path.append('../../../')
#from cpp_algorithms.common_helpers import imshow, imshow_scatter
#from bcd_helper import imshow, imshow_scatter
from shapely.geometry import Point, Polygon
from pathlib import Path
from pyproj import Transformer
# from zipfile import ZipFile
#from geopy import distance
#from skimage import measure
from skimage.draw import polygon
from shapely.geometry import Point, LineString, Polygon, MultiLineString,MultiPolygon
from shapely.ops import cascaded_union
import shapely
import shapely.geometry
from collections import defaultdict
from skimage.draw import polygon
import random 
import os
import re
import math
from time import sleep
from subprocess import PIPE, Popen
import subprocess
import json
import fiona
from shapely.geometry import shape 
import pandas as pd
import time
from FQ_GenTask import GetEFA
from FQ_Task_Generator import  TaskManager 
#from FQ_Task_Generator import TG, DetectFire, tasks, Timer,Grid, Phase2
#from FQ_TG_client import DrawTask
from mongotriggers import MongoTrigger 
from pymongo import MongoClient
####################################### Rxfire
def CreateRectangle(x_dl,y_dl, weight,height):# gag 10 m
    x_r=x_dl+weight
    y_u=y_dl+height
    coords=[(x_dl,y_dl), (x_r,y_dl), (x_r,y_u), (x_dl,y_u), (x_dl, y_dl)]
    poly=Polygon(coords)
    return poly 

def ClearBarrier(data):      
    polygons=[]
    for index, row in data.iterrows():
        poly = row['geometry']
        #print(f"seeeee {poly}")
        polygons.append(poly)
    #print(f"whyyyyy {polygons[0]}")
    u=cascaded_union([polygons[0],polygons[1],polygons[2]])
    u2=cascaded_union([polygons[3],polygons[6]])
    x,y=u2.exterior.coords.xy
    newdata = gpd.GeoDataFrame()
    newdata['geometry'] = None
    newdata.loc[0, 'geometry'] = u2.boundary[0]
    newdata.loc[1, 'geometry'] = polygons[4].boundary
    newdata.loc[2, 'geometry'] = u.boundary
    #da2=gpd.read_file(f"{dir}/{foldername}/input/seelin.shp")
    return newdata
    # da2.plot()
    # plt.show()    
def WriteInput(foldername, dir,step=1,initime=(0,0),dura=(2,0), dis_res=5, pre_res=5): # The simulation time 100 =1:00) duration<24 hours!
    fin = open(f"{dir}/{foldername}/input/Burn.input", "w")
    with open(f"{dir}/Template_burn/input/Burn.input", "r") as ftmp:
        filedata = ftmp.readlines()
    checkwind=False
    FARSITE_START_TIME=''
    for line in filedata:
        if re.match(r'FARSITE_START_TIME:*',line):
            time=line[:-1].split(' ')[-1]
            checkwind=True
            if initime[0]<10:
                time=f"0{initime[0]}"
            else:
                time=f"{initime[0]}"
            if initime[1]<10:
                time=time+f"0{initime[1]}"
            else:
                time=time+f"{initime[1]}"
            FARSITE_START_TIME=r'05 04 '+f"{time}"
            fin.write(f"FARSITE_START_TIME: {FARSITE_START_TIME}\n")
        elif checkwind  and re.match(r'2013 5 4 '+f"{time}",line):
            wind=int(line[:-1].split(' ')[-3])
            winddric=int(line[:-1].split(' ')[-2])
            fin.write(line)
        elif re.match(r'FARSITE_END_TIME:*',line):
            ent=[]
            ent.append(initime[0]+dura[0])
            if initime[1]+dura[1]>60:
                ent[0]=ent[0]+1
                ent.append(initime[1]+dura[1]-60)
            else:
                ent.append(initime[1]+dura[1])
            if ent[0]<10:
                time=f"0{ent[0]}"
            else:
                time=f"{ent[0]}"
            if ent[1]<10:
                time=time+f"0{ent[1]}"
            else:
                time=time+f"{ent[1]}"
            fin.write(f"FARSITE_END_TIME: 05 04 {time}\n")
        elif re.match(r'FARSITE_TIMESTEP:*',line):
            fin.write(f"FARSITE_TIMESTEP: {step}\n")
        elif re.match(f"FARSITE_DISTANCE_RES: ",line):
            fin.write(f"FARSITE_DISTANCE_RES: {float(dis_res)}\n")
        elif re.match(f"FARSITE_PERIMETER_RES: ",line):
            fin.write(f"FARSITE_PERIMETER_RES: {float(pre_res)}\n")
        else:
            fin.write(line)
    fin.close()


def PutSite(data, UID, Len, Width,foldername, dir, offset=20):
    if len(UID)==1:
        # Get Boundary: 
        # poly=data.loc[UID[0],'geometry']
        # poly=Polygon(poly)
        site=data.loc[UID[0],'geometry']
        x,y=site.coords.xy
        mx,my,bx,by=(min(x),min(y),max(x),max(y))
        #print(f"length: {max(x)-min(x)} width: {max(y)-min(y)}")
        #offset=20
        boundary=(int(min(x)+offset), int(min(y)+offset), int(min(x)+offset+Len), int(min(y)+offset+Width))
        mx,my,bx,by=boundary
        #poly=CreateRectangle(mx,my,Len,Width)
        coords=[(mx,my), (bx,my), (bx,by), (mx,by), (mx, my)]
        poly=LineString(coords)
        bound_space=(mx, my,bx,by)
        # if poly.geom_type=='Polygon':
        #     feature=[0]
        # else:[data.loc[i,'geometry'] for i in range(len(data))
        #     feature =[i for i in range(len(poly.geoms))] #
        bargdr = gpd.GeoDataFrame()
        bargdr['geometry'] = None
        bargdr.loc[0, 'geometry']=poly #, crs='EPSG:4326)
        # bargdr.plot()
        # plt.show()
        #print(f"bargdr {bargdr}")
    StartTestBed(foldername, bargdr, dir) # Create some folders for output
    return bound_space


def StartTestBed(foldername, bargdr, dir):
    os.system(f"mkdir {dir}/{foldername}")
    os.system(f"cp -r {dir}/Template_burn/input {dir}/{foldername}")
    bargdr.to_file(f"{dir}/{foldername}/input/TestBed.shp")    #------> This is the objective!!!!!! 
    try:
        os.system(f"rm  -r {dir}/{foldername}/output")
        os.system(f"mkdir {dir}/{foldername}/output")
    except:
        #print(f" did we remove the folder????" )
        os.system(f"mkdir {dir}/{foldername}/output")

def RunFarsite_Test(foldername, dir,time,simdur,step=1):   
    #write testfile
    tmp=[0,0]
    simdur=simdur+20  #I do not why it always stop earlier! 
    if simdur>=60:
        tmp[0]=simdur//60
        tmp[1]=simdur%60
    else:
        tmp[1]=simdur
    #print(f"Sim time {tmp}")
    WriteInput(foldername,dir,dura=tmp, step=step)
    f = open(f"{dir}/{foldername}/{foldername}_TEST.txt", "w")
    f.write(f"{dir}/{foldername}/input/Burn.lcp ")# write landscape
    f.write(f"{dir}/{foldername}/input/Burn.input ") # write input file
    f.write(f"{dir}/{foldername}/input/{foldername}.shp ")# write ignition fire
    f.write(f"{dir}/{foldername}/input/seelin.shp ")
    #f.write(f"{dir}/{foldername}/input/TestBed.shp ")   # We can change the barrier!!! 
    f.write(f"{dir}/{foldername}/output/{time} 0")   # Output
    f.close()
    #print(f"Command: {dir}/src/TestFARSITE {dir}/{foldername}/{foldername}_TEST.txt")
    try:
        out=os.system(f"{dir}/src/TestFARSITE {dir}/{foldername}/{foldername}_TEST.txt >/dev/null 2>&1")
        #print(f"see out", out.read())
        print(f"Generate the fire simulation successfully! Simulation from {time} duration  {simdur} ")
    except:
        print(f"Got error when simulating the fire spread")

def AddFireGrid(foldername, dir, prefix, time, poly):   # Update the ignition file! because we add some fire!!!! 
    collection = list(fiona.open(f"{dir}/{foldername}/output/{prefix}_{time}_Perimeters.shp",'r'))
    df1 = pd.DataFrame(collection)
    def isvalid(geom):
            if len(geom['coordinates'][0])>2:
                return 1
            else:
                return 0
    df1['isvalid'] = df1['geometry'].apply(lambda x: isvalid(x))
    df1 = df1[df1['isvalid'] == 1]
    collection = json.loads(df1.to_json(orient='records'))
    #Convert to geodataframe
    data = gpd.GeoDataFrame.from_features(collection)
    try:
        #print(f"{[data.loc[i, 'geometry'] for i in range(len(data))]}")
        geoms=[data.loc[i, 'geometry'] for i in range(len(data))]+[poly.loc[i, 'geometry'] for i in range(len(poly))]
        data2=shapely.ops.unary_union([geom if geom.is_valid else geom.buffer(0) for geom in geoms]) 
    except:
        print(f"some error")
        #data2=cascaded_union([data.loc[i, 'geometry'] for i in range(len(data))])
        geoms=[data.loc[i, 'geometry'] for i in range(len(data))]
        data2=shapely.ops.unary_union([geom if geom.is_valid else geom.buffer(0) for geom in geoms])
    try:
        features = [i for i in range(len(data2))]
        gdr = gpd.GeoDataFrame({'feature': features, 'geometry': data2}) #, crs='EPSG:4326)
    except:
        gdr = gpd.GeoDataFrame({'feature': [0], 'geometry': data2}) #, crs='EPSG:4326)
    gdr.to_file(f"{dir}/{foldername}/input/{foldername}.shp")    #------> This is the objective!!!!!! 
    # gdr.plot()
    # plt.show()

def EventUpdateEFA(Grid_fire, prefix, time,simdur,BunsiteBound,dir,fname,Res=1):
    mx,my,_,_=BunsiteBound
    poly = gpd.GeoDataFrame()
    i=0
    for g in list(Grid_fire.keys()):
        llx,lly=mx+g[0]*Res,my+g[1]*Res
        fire=CreateRectangle(llx,lly,Res,Res)
        poly.loc[i,'geometry']=fire
        i=i+1
    AddFireGrid(fname, dir, prefix, time, poly)
    RunFarsite_Test(fname, dir,time,simdur=simdur)
    EFA,EFAdict,bound = GetEFA(time,simdur,BunsiteBound,dir,fname,Res=Res) 
    mx,my,bx,by=np.array(BunsiteBound)//Res-np.array(bound)//Res
    bx=EFA.shape[0]+bx
    by=EFA.shape[1]+by
    EFA=EFA[int(mx):int(bx),int(my):int(by)]
    return EFA

####################################### Tow main functions!!!!! 
def UpdateFireIgnition(foldername, dir,BunsiteBound,Grid_map, Res, time,simdur, TM):   
    mx,my,_,_=BunsiteBound
    x,y=np.where(Grid_map==1)
    poly = gpd.GeoDataFrame()
    for i in range(len(x)):
        llx,lly=mx+x[i]*Res,my+y[i]*Res
        fire=CreateRectangle(llx,lly,Res,Res)
        poly.loc[i,'geometry']=fire
    gdr=poly
    gdr.to_file(f"{dir}/{foldername}/input/{foldername}.shp")    # Write fire ignition file!  
    RunFarsite_Test(foldername, dir,time,simdur=simdur) # Run the simulation 
    EFA,EFAdict,bound = GetEFA(inittime,simdur,BunsiteBound,dir,foldername,Res=Res) # Read the output, get EFA
    mx,my,bx,by=np.array(BunsiteBound)//Res-np.array(bound)//Res
    bx=EFA.shape[0]+bx
    by=EFA.shape[1]+by
    EFA=EFA[int(mx):int(bx),int(my):int(by)]
    tasks=TM.DeclareGridEFA(EFA,init=inittime) # Generate tasks 
    
    return EFA,tasks


def UpdateReportFire(Grid_fire,simdur,BunsiteBound,dir,foldername,Res, TM):
    tasks, Update,prefix,time=TM.ReportFire(Grid_fire)
    if Update: # Get new 
        #nEFA=EventUpdateEFA(Grid_fire, prefix, time,simdur,BunsiteBound,dir,foldername,Res=Res)
        
        mx,my,_,_=BunsiteBound
        poly = gpd.GeoDataFrame()
        i=0
        for g in list(Grid_fire.keys()):
            llx,lly=mx+g[0]*Res,my+g[1]*Res
            fire=CreateRectangle(llx,lly,Res,Res)
            poly.loc[i,'geometry']=fire
            i=i+1
        AddFireGrid(foldername, dir, prefix, time, poly) # Create the ignition file
        RunFarsite_Test(foldername, dir,time,simdur=simdur) #Run the simulation 
        EFA,EFAdict,bound = GetEFA(time,simdur,BunsiteBound,dir,foldername,Res=Res) # Read output, and EFA
        mx,my,bx,by=np.array(BunsiteBound)//Res-np.array(bound)//Res
        bx=EFA.shape[0]+bx
        by=EFA.shape[1]+by
        EFA=EFA[int(mx):int(bx),int(my):int(by)]
        
        TM.latime=time
        tasks=TM.EventUpdateFT(EFA,time) # Generate tasks! 
    else:
        EFA=[]
    return Update, EFA,tasks
    
Phase2=False

    
if __name__ == "__main__":
    dir='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite/Rxfire/Burn_1/output/Burn_1'
    igfile='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite/Rxfire/Burn_1/input/FQburn'
    dir='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite'
    file='CARB_BurnUnits/CARB_BurnUnits.shp'
    data=gpd.read_file(f"{dir}/{file}")
    foldername='FQ_test'
    data=ClearBarrier(data)
    Missions=defaultdict(dict)  #  Missions: period, priority, 
    Missions['BM']=[ 10, 1]  #Other area 
    Missions['FI']=[ 5, 1]  #track the fire 
    Missions['FT']=[ 3, 3]  # track the fire perimeter and the risky area (arrival within 10 min)
    Missions['FL']=[ 3, 2]  # FL is tracking the fire perimeter.
    Missions['FD']=[ 5, 3]
    Res=2.5 # Grid size
    Len=8*2
    Width=6*2
    simdur=120
    TM=TaskManager(Missions,theta=9, plantime=simdur) # Create the task manager! 

    client = MongoClient("mongodb://127.0.0.1:27017/")
    
    #client = MongoClient("mongodb://169.234.54.191:27017/")

    
    Statetrigger = MongoTrigger(client)
    firedb = client["fireMap"]
    # # Create Collection (table) called grids
    gridCollection = firedb.gridStates
    EFACollection = firedb.wpEFA

    statelist=[(int(x['State']), int(x['Time'])) for x in gridCollection.find()]
    State_dict=dict(zip(list(range(1,len(statelist)+1)),statelist))
    Grid_map=np.full((int(Len/2),int(Width/2)),0)
    Unvisited=list(range(1,len(statelist)+1))
    timeStep=3
    BunsiteBound=PutSite(data, [2], Len*Res,Width*Res,foldername, dir) # Create folders and input files! 
    # latime=0
    # cutime=0
    inittime=0

    
    def tryInsert(op_document): 
        print(f"see get state {op_document} ")
        print([i for i in gridCollection.find({'_id':op_document['o2']['_id'] })][0])
        '''
        Update=False
        id=int(op_document['o']['Grid ID'])
        state=int(op_document['o']['State'])
        time=int(op_document['o']['Time'])
        if State_dict[id][1]!=time:
            State_dict[id]=(State_dict[id][0],time)
            cutime=time
            if not Phase2:
                try:
                    Unvisited.remove(id)
                except:
                    pass
        if State_dict[id][0]!=state:
            State_dict[id]=(state,State_dict[id][1])
            print(f"update  id {id} to state {state}")
        if not Phase2:
            if len(Unvisited)==0:
                Phase2=True
                cutime=0
                latime=0
                print(f"see state {State_dict}")
                Update=True
        else:
            if cutime>latime+timeStep:
                latime=cutime
                Update=True
                print(f"see need to do event-driven ")
        #return Update
        '''
    def tryUpate(op_document): 
        global Phase2
        global BunsiteBound
        global Res
        global simdur
        global TM
        print(f"see get state {Phase2} {op_document['o2']['_id']}")
        print([i for i in gridCollection.find({'_id':op_document['o2']['_id'] })][0])
        getit=[i for i in gridCollection.find({'_id':op_document['o2']['_id'] })][0]
        Update=False
        id=int(getit['Grid ID'])
        state=int(getit['State'])
        time=int(getit['Time'])
        x,y=getit['Cords']
        
        if State_dict[id][1]!=time:
            State_dict[id]=(State_dict[id][0],time)
            
            cutime=time
            if not Phase2:
                try:
                    Unvisited.remove(id)
                except:
                    pass
        if State_dict[id][0]!=state:
            State_dict[id]=(state,State_dict[id][1])
            #print(f"check {(x,y)}")
            Grid_map[int(y),int(x)]=state
            #print(f"update  id {id} to state {state}")
        if not Phase2:
            if len(Unvisited)==0:
                Phase2=True
                cutime=0
                latime=0
                #print(f"see state {State_dict}")
                #print(f"see Grid map {Grid_map}")
                EFA, tasks=UpdateFireIgnition(foldername, dir, BunsiteBound,Grid_map, Res, inittime,simdur, TM)    
                print(EFA)
                #Write the EFA to EFA
                r,w=Grid_map.shape
                print(f"check EFA {[i for i in EFACollection.find()]}")
            
                for i in range(r):
                    for j in range(w):
                        print(i,j, EFA[j,j])
                        get=[i for i in EFACollection.find({'Cords': (j,i)})][0]
                        EFACollection.update_one({'Cords': (j,i)}, {'$set': {'Grid ID': int(get['Grid ID']),'Cords': (j,i),'EFA': int(EFA[j,i])}})
                print([i for i in EFACollection.find({'Cords': (j,i)})][0])         
                            
                
                
                
                
                
                print(tasks)
                #DrawTask(tasks,EFA) 
                
                Update=True
        else:
            if cutime>latime+timeStep:
                latime=cutime
                Update=True
                print(f"see need to do event-driven ")
        #return Update
        #Phase2=True
    Statetrigger.register_insert_trigger(tryUpate, 'fireMap', 'gridStates')
    Statetrigger.register_update_trigger(tryUpate, 'fireMap', 'gridStates')
    #Statetrigger.register_op_trigger(tryUpate, 'fireMap', 'gridStates')
    Statetrigger.tail_oplog()
    print(f" is it over")
    print(Statetrigger)
    # while True:
    #     pass
    #Statetriggers.stop_tail()

    '''
    
    #Example 1: 
    Grid_map=np.full((8,6),0)
    Grid_map[0,1]=1
    Grid_map[1,1]=1
    Grid_map[3,1]=1
    Grid_map[2,1]=1 
    ##### Get the fire simulation!!! 
    EFA, tasks=UpdateFireIgnition(foldername, dir, BunsiteBound,Grid_map, Res, inittime,simdur, TM)    
    DrawTask(tasks,EFA) 
    
    #Example 2:
    Grid_fire={(0,6):10, (2,3):10}
    
    Update, nEFA, tasks =UpdateReportFire(Grid_fire,simdur,BunsiteBound,dir,foldername,Res, TM)
    DrawTask(tasks,EFA) 
    
    '''
    
    
    
    
    
    #DeclareGrid(TG, EFA,init=1, tasks=tasks,Missions=Missions)

########################################### Old code, for something else. 