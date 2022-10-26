
import numpy as np
import matplotlib.pyplot as plt
import sys
import geopandas as gpd
sys.path.append('../../../')
#from cpp_algorithms.common_helpers import imshow, imshow_scatter
#from bcd_helper import imshow, imshow_scatter
from shapely.geometry import Point, Polygon
from pathlib import Path
# from zipfile import ZipFile
#from geopy import distance
#from skimage import measure
import math
from skimage.draw import polygon
import shapely
import shapely.geometry
from collections import defaultdict
from skimage.draw import polygon
import os
import random
import json 
import fiona
import pandas as pd
#from FQ_TaskAllocation import Decomposition, Auction, TrackDraw, Drone, Voronoi

def get_hv_wh(BunsiteBound):
    """
    Get haversine calcualted width and height of
    the smallest bounding rectangle of the coverage area.
    """
    llng, llat, rlng, rlat =  BunsiteBound
    w=abs(llng-rlng)
    h=abs(llat-rlat)
    print(f"FQ Weight: {w}; Height: {h}")
    return w, h
"""
Convert to raster
"""
def get_scale(BunsiteBound, meter=1):
    """
#     Returns the supposed longside of the area.
#     """
    w,h = get_hv_wh(BunsiteBound)
    return int(np.round((np.array([w,h])/meter).max()))

def get_raster(gpdf_final, scale=2000, BunsiteBound=0, NF=-1, F=1, P=2, Res=1):
    """
    Returns rasterised version for the projection
    """
    polys=[gpdf_final.loc[i, 'geometry'].exterior.coords for i in range(len(gpdf_final))]
    minx=min([min(np.array(i).T[0]) for i in polys])
    maxx=max([max(np.array(i).T[0]) for i in polys])
    miny=min([min(np.array(i).T[1]) for i in polys])
    maxy=max([max(np.array(i).T[1]) for i in polys])
    boundary=[min(BunsiteBound[0],minx),min(BunsiteBound[1],miny),max(BunsiteBound[2],maxx),max(BunsiteBound[3],maxy)]
    #print(f"see pre boundary {np.array(boundary)}")
    #bound=np.array(boundary)//Res
    mn=np.array(boundary[:2])
    sh=(np.array(boundary[2:])//Res-mn//Res)+np.array([1,1])# FQ change the boundary #I changed the boundary for perimeter!!!1
    # sh[0]=math.ceil(sh[0])
    # sh[1]=math.ceil(sh[1])
    sh=np.int64(sh)
    p = np.full(sh,NF) #FQ change that
    empty=np.full(sh,NF) #for record perimeter
    #################I forgot why I want to get ceiling and floor?????# 
    #But at least, I finished the EFA, and simulation!....... 
    for i in range(len(gpdf_final)):
        shp=gpdf_final.loc[i, 'geometry']
        ext = np.array(shp.exterior.coords)
        x,y=ext.T
        mix = (ext//Res - mn//Res)
        mix1 = np.int64(mix)
        r,c = polygon(*mix1.T,sh)   #fq? should reverse that? 
        p[r,c] = F
        r,c=mix1.T # for the fire perimeter 
        #print(f"why {r} {c}")
        try:
            empty[r,c]=0
        except:
            print(f"enter")
            print(f"empty {empty.shape}")
            plt.plot(r,c )
            plt.show()
        #p[r,c]=P    #------------------------------------> Here, I found if uncomment this, we will get more fire????
        # mix1=np.ceil(mix/Res)#*3
        # mix1 = np.int64(mix1)
        # r,c = polygon(*mix1.T,sh)   #fq? should reverse that? 
        # p[r,c] = F
        # # imshow(p)
        # # plt.show()
        # r,c=mix1.T # for the fire perimeter 
        # #print(r,c)
        # empty[r,c]=0
        # p[r,c]=P
        # mix2=np.floor(mix/Res)#*30
        # mix2=np.int64(mix2)
        # r,c = polygon(*mix2.T,sh)
        # p[r,c] = F
        # r,c = mix2.T
        # p[r,c] = P 
        # empty[r, c]=0
    x,y=np.where(empty==0)
    # print(f"see p")
    # imshow(p)
    #plt.show()
    
    #print(f"FQ change to  {len(x)}")
    
    return p,[x,y],boundary


def Firefront(maps):
    cu_map=maps[0]
    fu_map=maps[1]
    return cu_map-fu_map

def logEFA(EFAdict,Primdict, filename, shape, Res):
    f = open(filename, "w") 
    print(shape[0])
    f.write(f"{shape[0]} {shape[1]}\n ")
    for key in EFAdict.keys():
        row=np.array(EFAdict[key])
        f.write(f"{key} {len(list(row[0]))} ")
        for i in list(row[0]):
            f.write(f"{i},")
        f.write(f" ")
        for i in list(row[1]):
            f.write(f"{i},")
        f.write(f"\n")
        f.write(f"Prim {len(list(Primdict.get(key)[0]))} ")
        for j in list(Primdict.get(key)[0]):
            f.write(f"{j},")
        for j in list(Primdict.get(key)[1]):
            f.write(f"{j},")
        f.write(f"\n")
    f.close() 


def PutIginition(filename,dir):
    tofile=f"{dir}_0_Perimeters"
    c1=f"cp {filename}.shp {tofile}.shp"
    c2=f"cp {filename}.shx {tofile}.shx"
    os.system(c1)
    os.system(c2)
    print(f"Run {c1}")

def UpadteEFA(Pmap, area_map,time,EFA, prim_bound,bound, NF=-1, F=1, P=2, Res=10): #    Get the estimated fire arrival time.
    if len(Pmap)==1:
        img=area_map
        EFA = np.full(img.shape, -1,dtype=np.uint8)
        
        EFA[img == P] = time
        EFA[img == F] = time
        # imshow(EFA)
        # plt.show()
        x,y= np.where(EFA==time)
    else:
        #################################################
        #print(f"check at beginning {Pmap.shape} {area_map.shape} {prim_bound} {bound} ")
        prim_bound=np.int64(np.array(prim_bound)//Res)
        bound=np.int64(np.array(bound)//Res)
        #print(f"check at beginning {Pmap.shape} {area_map.shape} {prim_bound} {bound} ")
        if list(prim_bound)!=list(bound):
            new_bound=[min(prim_bound[0],bound[0]),min(prim_bound[1],bound[1]), max(prim_bound[2],bound[2]),max(prim_bound[3],bound[3])]  
            if prim_bound[0]>new_bound[0]:    # previous minx is smaller.
                #print(f"check prim_bound minx {prim_bound[0]} {Pmap.shape}")
                tmprow=[np.array([-1]*Pmap.shape[1])]
                for i in range(prim_bound[0]-new_bound[0]):
                    Pmap=np.insert(Pmap,0,tmprow, axis=0)
                    #print(f"check prim_bound minx {Pmap.shape} {area_map.shape} ")
                    EFA=np.insert(EFA,0,tmprow, axis=0)  
                #print(f"check final miny {Pmap.shape} {area_map.shape}")
            if prim_bound[1]>new_bound[1]: 
                #print(f"check prim_bound miny {Pmap.shape} {area_map.shape}")
                tmpcolumn=[np.array([-1]*Pmap.shape[0])]
                for i in range(prim_bound[1]-new_bound[1]):
                    Pmap=np.insert(Pmap,0,tmpcolumn, axis=1)
                    EFA=np.insert(EFA,0,tmpcolumn, axis=1)
                #print(f"check final miny {Pmap.shape} {area_map.shape}")
            if prim_bound[2]<new_bound[2]: 
                tmprow=[np.array([-1]*Pmap.shape[1])]
                #print(f"check prim_bound maxx {Pmap.shape} {area_map.shape}")
                for i in range(new_bound[2]-prim_bound[2]):
                    Pmap=np.insert(Pmap,len(Pmap),tmprow, axis=0)
                    EFA=np.insert(EFA,len(EFA),tmprow, axis=0)
            if prim_bound[3]<new_bound[3]: 
                #print(f"check prim_bound maxy {Pmap.shape} {area_map.shape}")
                tmpcolumn=[np.array([-1]*Pmap.shape[0])]
                #print(new_bound[3]-prim_bound[3])
                for i in range(new_bound[3]-prim_bound[3]):
                    Pmap=np.insert(Pmap,len(Pmap[0]),tmpcolumn, axis=1)
                    EFA=np.insert(EFA,len(EFA[0]),tmpcolumn, axis=1)
        #print(f"shape area_map {area_map.shape} {Pmap.shape}")
        #tmp=area_map-Pmap     # Get first arrival time. 
        #imshow(area_map)
        tmp1=np.full(Pmap.shape,0)
        tmp1[area_map>=0]=1
        tmp2=np.full(Pmap.shape,0)
        tmp2[Pmap>=0]=1
        tmp=tmp1-tmp2
        tmp[EFA!=255]=0   # Here I want to grid already burned, 
        # imshow(Pmap)
        # imshow(tmp)
        #     imshow(area_map)
        #     plt.show()
        EFA[tmp>0]=time
        x,y=np.where(EFA==time)
    return EFA, x,y
  
def GetEFFNext(init,end, BunsiteBound, dir, foldername, Res=10):
    scale = get_scale(BunsiteBound, meter=1)
    EFAdict=defaultdict(list)
    timestamp=3
    time=init
    Primdict=defaultdict(list)
    Pmap=[0]
    EFA=[]  # EFA is matrix shows the expected fire arrival time
    prim_bound=BunsiteBound
    while time<=end:
        file=f"{dir}/{foldername}/output/{foldername}_{time}_Perimeters.shp"
        data=gpd.read_file(file)
        # data.plot()
        # plt.show()
        #print(f"see time {time}")
        area_map_,prim,bound= get_raster(data, scale, prim_bound, NF=-1, F=1, P=2, Res=Res)
        EFA,x,y=UpadteEFA(Pmap, area_map_,time,EFA,prim_bound,bound)
        prim_bound=bound
        EFAdict[time]=[x,y]
        Primdict[time]=prim
        Pmap=area_map_ 
        time=time+timestamp
    #shape=area_map_.shape
    #logEFA(EFAdict, Primdict,"EFAdict.csv",shape, 10)
    return EFA, EFAdict, Primdict

def GetEFA(init,simdur, BunsiteBound, dir, foldername,step=3, Res=10):
    def isvalid(geom):
        if len(geom['coordinates'][0])>2:
            return 1
        else:
            return 0
    scale = get_scale(BunsiteBound, meter=1)
    EFAdict=defaultdict(list)
    time=0
    Primdict=defaultdict(list)
    Pmap=[0]
    EFA=[]  # EFA is matrix shows the expected fire arrival time
    prim_bound=0
    while time<=simdur:
        if time==0:
            file=f"{dir}/{foldername}/input/{foldername}.shp"
        else:
            file=f"{dir}/{foldername}/output/{init}_{time}_Perimeters.shp"
        collection = list(fiona.open(file,'r'))
        df1 = pd.DataFrame(collection)
        df1['isvalid'] = df1['geometry'].apply(lambda x: isvalid(x))
        df1 = df1[df1['isvalid'] == 1]
        collection = json.loads(df1.to_json(orient='records'))
        data = gpd.GeoDataFrame.from_features(collection)
        # data.plot()
        # plt.show()
        area_map_,prim,bound= get_raster(data, scale, BunsiteBound,NF=-1, F=1, P=2, Res=Res)
        EFA,x,y=UpadteEFA(Pmap, area_map_,time,EFA,prim_bound, bound, Res=Res)
        if time==init:
            IniPerimeter=prim   #-----> Check, if we need the perimeter later????
        prim_bound=bound
        EFAdict[time]=[x,y]
        Primdict[time]=prim
        Pmap=area_map_ 
        time=time+step 
    #shape=area_map_.shape
    #logEFA(EFAdict, Primdict,"EFAdict.csv",shape, 10)
    return EFA, EFAdict, bound #, IniPerimeter

class Sensor:
    def _init_(self, Type='ALL', Pixel=1080, AFoV=math.pi*(60/180)):
        pass
    

if __name__ == "__main__":
    Missions=defaultdict(dict)  #  Missions: period, priority, 
    Missions['BM']=[ 10, 1]  #Other area 
    Missions['FI']=[ 5, 1]  #track the fire 
    Missions['FT']=[ 3, 3]  # track the fire perimeter and the risky area (arrival within 10 min)
    Missions['FL']=[ 3, 2]  # FL is tracking the fire perimeter.
    Missions['FD']=[ 2, 3]
    ################ Get tasks.
    dir='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite'
    foldername='FQ_burn'
    BunsiteBound=(701235.0,4308525.0,704355.0,4311675.0)
    BunsiteBound=(702374.0,4309425.0,703700.0,4310900.0)
    Bursite=(702460.0,4309700.0,703540.0,4310820.0 )
    # igfile='/home/fangqiliu/eclipse-workspace_Python/Drone_path/CoveragePathPlanning-master/farsite/Rxfire/Burn_1/input/FQburn'
    # PutIginition(igfile,dir)
    init=120
    end=150
    Res=10
    EFA,EFAdict,Primdict =GetEFA(init,end,Bursite,dir,foldername, Res=Res)
    #Here we need use EFA to input them into the task generator. 
    # imshow(EFA)
    # plt.show()
    print(f"EFA: {EFA}")
    print(f"EFAdict: {EFAdict}")
    print(f"Primdict: {Primdict}")
    #Task_mission=GenTasks(init,end,Bursite,dir,foldername, Missions ,Res=Res)
    #print(Task_mission)
























