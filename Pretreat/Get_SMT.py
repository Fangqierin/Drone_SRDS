'''
Created on May 25, 2020

@author: fangqiliu
'''
import itertools
from collections import defaultdict
import copy
import matplotlib.pyplot as plt 
import numpy as np
import random
import time      
class Agent(): 
    def __init__(self,id,nodeset,M_dis=None,M_fre=[],To=5): 
        self.id=id
        self.id_set=nodeset
        self.V = len(nodeset)
        self.MT = []   
        self.leaf=[]
        self.length=0
        self.to_t=0
        self.To=To
        self.tsp=[]
        self.M_dis=M_dis
        self.M_fre=M_fre
    # A utility function to print the constructed MST stored in parent[] 

    def minKey(self, key, mstSet): 
        min = float('inf')
        for v in range(self.V): 
            if key[v] < min and mstSet[v] == False: 
                min = key[v] 
                min_index = v 
  
        return min_index 
    # Function to construct and print MST for a graph  
    # represented using adjacency matrix representation 
    def primMST(self):  # Run at the first time
        # Key values used to pick minimum weight edge in cut 
        nodeset=copy.deepcopy(self.id_set)
        if len(self.M_fre)!=0:
            self.to_t=sum([self.M_fre[i]*self.To for i in nodeset])   # travel time plus loitor time   
        if len(self.M_dis) != 0:
            self.graph = [[0 for i in range(len(nodeset))] for j in range(len(nodeset))] 
            for i in range(len(nodeset)):
               # print(nodeset)
                for j in range(i+1,len(nodeset)):
                    self.graph[i][j]=self.M_dis[nodeset[i]][nodeset[j]]
                    self.graph[j][i]= self.graph[i][j]
        
        key = [float('inf')] * self.V 
        parent = [None] * self.V # Array to store constructed MST 
        # Make key 0 so that this vertex is picked as first vertex 
        key[0] = 0 
        mstSet = [False] * self.V
        parent[0] = -1 # First node is always the root of 
        for cout in range(self.V): 
            u = self.minKey(key, mstSet) 
            mstSet[u] = True
            for v in range(self.V): 
                if self.graph[u][v] > 0 and mstSet[v] == False and key[v] > self.graph[u][v]: 
                        key[v] = self.graph[u][v] 
                        parent[v] = u 
        summ=0;MT=[]
        for i in range(1, self.V): 
            summ=summ+self.graph[i][ parent[i] ]
            #print (self.id_set[parent[i]], "-", self.id_set[i], "\t", self.graph[i][ parent[i] ],self.M_dis[self.id_set[parent[i]]][self.id_set[i]] )
            #print(parent[i], "-", [i])
            MT.append([parent[i],i,self.graph[i][ parent[i] ]])
        self.MT=MT
        e=list(itertools.chain.from_iterable([i[:-1] for i in self.MT]))
        tm= [i for i in e if e.count(i)==1]
        #print(f"www {self.MT}")
        self.leaf=[self.id_set[i] for i in tm]
        self.length=sum(i[2] for i in self.MT)
        self.to_t=self.to_t+self.length      
    ########## for KruskalMST
    def find(self, parent, i):
       # print(f"why {len(parent)},{parent} {i}")
        if parent[i] == i:
            return i
        return self.find(parent, parent[i])
    def union(self, parent, rank, x, y):
        xroot = self.find(parent, x)
        yroot = self.find(parent, y)
        if rank[xroot] < rank[yroot]:
            parent[xroot] = yroot       
        elif rank[xroot] > rank[yroot]:
            parent[yroot] = xroot 
        else :
            parent[yroot] = xroot   
            rank[xroot] += 1
#         print(f"see {parent}")
    def get_tsp(self):
        nodes=[]
        dic_child=defaultdict(list)
        for i in self.MT:
            if i[0] in nodes:
                if i[1] not in nodes:
                    dic_child[i[0]].append(i[1])
            else:
                if i[1] in nodes:
                    if i[1] not in nodes:
                        dic_child[i[1]].append(i[0])
                else:
                    dic_child[i[0]].append(i[1])
        def preorder(i):  
            self.tsp.append(i)   
            if dic_child.get(i)!=None: 
                for i in dic_child.get(i):
                    preorder(i)
        preorder(0)
        self.tsp.append(0)
        dis_tsp=0
        c=0
        while c< len(self.tsp)-1:
            a1=self.id_set[self.tsp[c]]
            a2=self.id_set[self.tsp[c+1]]
            dis_tsp=dis_tsp+self.M_dis[a1][a2]
            #print(f"add {a1}-{a2}")
            c=c+1
        self.dis_tsp=dis_tsp


graph = [ [0, 2, 0, 6, 0], 
            [2, 0, 3, 8, 5], 
            [0, 3, 0, 0, 7], 
            [6, 8, 0, 0, 9], 
            [0, 5, 7, 9, 0]] 
# g = Agent(0,[2,3,1,0,4],graph) 
# g.primMST()
# g.get_tsp()
# print(g.MT)
# 
# g.addNode([])
def Greedy_SMT(case,Cu_Wap_ty,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm):
    TSP=[];w_id=w_in; cu_t=T_in; P=[w_in]
    if case==11:
    #if kind==-1:
        Wap_set=list(itertools.chain(*Cu_Wap_ty))
        id_set=[w_in]+[Wap_set[i].id for i in range(len(Wap_set))]
        g=Agent(w_in,id_set,D)
        g.primMST()
        g.get_tsp()
        i=1
        seq=g.id_set[g.tsp[0]]
        while cu_t<=T_tm:
            P.append(g.id_set[g.tsp[i]])
            cu_t=cu_t+D[seq][g.id_set[g.tsp[i]]]
            d_l=D[seq][g.id_set[g.tsp[i]]]
            seq=g.id_set[g.tsp[i]]
            i=i+1
            if i==len(g.tsp)-2:
                if w_in==0:
                    i=1
                else:
                    i=0
        return P[:-1],cu_t-d_l
    else:
        a=random.random()
        if a>0.7:
            f=0
        else:
            f=1
        if T_in<=300:
            f=1
        Wap_set=list(Cu_Wap_ty[f])
        id_set=[w_in]+[Wap_set[i].id for i in range(len(Wap_set))]
        g=Agent(w_in,id_set,D)
        g.primMST()
        g.get_tsp()
        p1=[g.id_set[g.tsp[i]] for i in range(len(g.tsp))]
        la_w=p1[-2]
        if f==1:
            f=0
        else:
            f=1
        Wap_set=list(Cu_Wap_ty[f])
        id_set=[la_w]+[Wap_set[i].id for i in range(len(Wap_set))]
        g=Agent(la_w,id_set,D)
        g.primMST()
        g.get_tsp()
        p2=p1[:-1]+[g.id_set[g.tsp[i]] for i in range(len(g.tsp))][1:-1]
        i=1;seq=p2[0]
        while cu_t<=T_tm:
            P.append(p2[i])
            cu_t=cu_t+D[seq][p2[i]]
            dl=D[seq][p2[i]]
            seq=p2[i]
            i=i+1
            if i==len(p2)-1:
                if w_in==0:
                    i=1
                else:
                    i=0
        return P[:-1],cu_t-dl
    #I want to see result [0, 444, 445, 443, 442, 447, 452, 451, 387, 389, 391, 455, 453, 388, 390, 392, 397, 396, 395, 394, 399, 398, 454, 456, 17, 18, 19, 20, 24, 28, 63, 62, 61, 64, 67, 79, 80, 81, 84, 83, 82, 65, 68, 66, 69, 23, 27, 22, 26, 21, 25, 41, 42, 43, 44, 48, 47, 46, 45, 377, 378, 379, 380, 384, 383, 382, 381, 353, 354, 355, 356, 315, 314, 313, 316, 319, 331, 332, 333, 336, 335, 334, 317, 320, 318, 321, 360, 364, 359, 363, 358, 362, 357, 361, 17]

    #while cu_t<T_tm:
        
#         while True:
#             if len(id_set)>0:
#                 Dis=sorted([(D[w_id][w],w) for w in id_set])
#                 w_c=Dis[0][1]
#                 id_set.remove(w_c)
#                 TSP.append(w_c)
#                 w_id=w_c
#             else:
#                 if num==1: num=0 
#                 else: break
#                 id_set=[Wap_set[num][i].id for i in range(len(Wap_set[num]))]
#     else:
#         id_set=[Wap_set[i].id for i in range(len(Wap_set))]
#         while len(id_set)>0:
#             Dis=sorted([(D[w_id][w],w) for w in id_set])
#             w_c=Dis[0][1]
#             id_set.remove(w_c)
#             TSP.append(w_c)
#             w_id=w_c
#     seq=0; w_id=w_in
#    # print(f"TSP {TSP}")
#     while cu_t<T_tm:
#         if seq<len(TSP):
#             #  sP.append(TSP[seq])
#             cu_t=cu_t+D[w_id][TSP[seq]]
#             if cu_t>=T_tm:
#                 cu_t=cu_t-D[w_id][TSP[seq]]
#                 break
#             P.append(TSP[seq])
#             w_id=TSP[seq]
#             seq=seq+1; 
#         else: 
#             seq=0     
#     return(P,cu_t)


######################################################  Kruskal Minimum Spanning Tree Algorithm

