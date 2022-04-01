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
class Groups():
    def __init__(self,id_set,Centers,M_dis,M_fre,dic_M_W,To):
        self.M_dis=M_dis
        self.M_fre=M_fre
        self.Center=Centers
        self.id_set=id_set
        self.Drone_num=len(Centers)
        self.dic_M_W=dic_M_W
        self.To=To
        self.M_set_split=[[] for i in range(len(Centers))]
        self.to_t=[[] for i in range(len(Centers))]
    def draw (self,M_set_split,Center):
        co=['y','g','orange','black','b']
        fig=plt.figure()
        ax=fig.add_subplot(111,projection='3d')
    #             ax.scatter([M_loc[i][0] for i in range(len(M_loc))],[M_loc[i][1] for i in range(len(M_loc))],
    #                        [M_loc[i][2] for i in range(len(M_loc))])
        for i in range(len(M_set_split)):
           #print(i)
            ax.scatter([self.dic_M_W.get(i)[0] for i in M_set_split[i]],[self.dic_M_W.get(i)[1] for i in M_set_split[i]],[self.dic_M_W.get(i)[2] for i in M_set_split[i]],color=co[i])
        ax.scatter([self.dic_M_W.get(i)[0] for i in Center],[self.dic_M_W.get(i)[1] for i in Center],[self.dic_M_W.get(i)[2] for i in Center],color='r',s=50)
        plt.show()               
    def k_Medoid(self):
        Av_center=copy.deepcopy(self.Center)
        re_id=list(set(self.id_set)-set(self.Center))
        M_set_split=[[] for i in range(len(self.Center))]
        m_w=dict(zip(list(range(self.Drone_num)),[0]*self.Drone_num))
        for i in range(self.Drone_num):
            M_set_split[i].append(self.Center[i])
            m_w[i]=m_w[i]+self.M_fre.get(self.Center[i])
        for i in re_id:
            dis=sorted([(self.M_dis[i][Av_center[j]],j) for j in range(len(Av_center))])
            d_check=[dis[i][1] for i in range(len(dis)) if dis[i][0]==dis[0][0]]
        #D=sorted([(min(M_dis[i][j] for j in Av_center),i) for i in T_M-set(Av_center)])
            if len(d_check)>1:
                ind=sorted([(m_w[i],i) for i in d_check])[0][1]  
            else:
                ind=d_check[0]
            M_set_split[ind].append(i)
            m_w[ind]=m_w[ind]+self.M_fre[i]
        self.M_set_split=M_set_split
        #self.draw(M_set_split,self.Center)
    def get_center(self):
        old_center=copy.deepcopy(self.Center)
        Dis=[[] for i in range(len(self.M_set_split))]
        seq=0
        new_center=[]
        new_to_t=[0]*len(self.M_set_split)
        for i in range(len(self.M_set_split)):
            D=np.zeros((len(self.M_set_split[i]),len(self.M_set_split[i])))
            id=self.M_set_split[i]
            for a in range(len(id)):
                for b in range(len(id)):
                    if a!=b:
                        D[a][b]=self.M_fre[id[b]]*self.M_dis[id[a]][id[b]]
            dis=sorted([(sum(D[i][:]),i) for i in range(len(D))])
            new_to_t[i]=dis[0][0]+sum([self.M_fre[id[k]]*self.To for k in range(len(id))])
            ind=dis[0][1]
            new_center.append(id[ind])
        if new_center!=old_center:
            print(f"not equal, {new_center}, {old_center}")
        self.to_t=new_to_t
        self.Center=new_center
        return new_center
    def Get_initial(self):
        self.k_Medoid()
        new_center=self.get_center()
        while new_center!=self.Center:
            self.Center=new_center
            self.k_Medoid()
            new_center=self.get_center()
#     def transfer(self,node,agent1,agent2):
#         M_set_split=copy.deepcopy(self.M_set_split)
#         Center=copy.deepcopy(self.Center)
#         to_t=copy.deepcopy(self.to_t)
#         M_set_split[agent1].append(node)
#         M_set_split[agent2].remove(node)
#         for i in [agent1,agent2]:
#             D=np.zeros((len(M_set_split[i]),len(self.M_set_split[i])))
#             id=self.M_set_split[i]
#             for a in range(len(id)):
#                 for b in range(len(id)):
#                     if a!=b:
#                         D[a][b]=self.M_fre[id[b]]*self.M_dis[id[a]][id[b]]
#             dis=sorted([(sum(D[i][:]),i) for i in range(len(D))])
#             to_t[i]=dis[0][0]+sum([self.M_fre[id[k]]*self.To for k in range(len(id))])
#             ind=dis[0][1]
#             Center[i]=ind
#         print(f" transfer {node} from {agent1} to {agent2} change to {self.to_t}")
#         return to_t, M_set_split,Center
#     def swap(self,node1,node2,agent1,agent2):
#         M_set_split=copy.deepcopy(self.M_set_split)
#         Center=copy.deepcopy(self.Center)
#         to_t=copy.deepcopy(self.to_t)
#         M_set_split[agent1].remove(node1)
#         M_set_split[agent1].append(node2)
#         M_set_split[agent2].remove(node2)
#         M_set_split[agent1].append(node1)
#         for i in [agent1,agent2]:
#             D=np.zeros((len(M_set_split[i]),len(self.M_set_split[i])))
#             id=self.M_set_split[i]
#             for a in range(len(id)):
#                 for b in range(len(id)):
#                     if a!=b:
#                         D[a][b]=self.M_fre[id[b]]*self.M_dis[id[a]][id[b]]
#             dis=sorted([(sum(D[i][:]),i) for i in range(len(D))])
#             to_t[i]=dis[0][0]+sum([self.M_fre[id[k]]*self.To for k in range(len(id))])
#             ind=dis[0][1]
#             Center[i]=ind
#         print(f" swap {node1} with {node2} from {agent1} to {agent2} change to {self.to_t}")
#         return to_t, M_set_split,Center
#          
class Partitioner():
    def __init__(self,Groups,M_dis,M_fre,Drone_num,dic_M_W,To,Max_time,Max_iter):
        self.Groups=Groups
        self.M_dis=M_dis
        self.M_fre=M_fre
 #       self.id_set=id_set
        self.Drone_num=Drone_num
        self.dic_M_W=dic_M_W
        self.To=To
        self.Max_time=Max_time 
        self.Max_iter=Max_iter
 #       self.M_set_split=[[] for i in range(len(Centers))]
 #       self.to_t=[[] for i in range(len(Centers))]
    def transfer(self,Groups,node,agent1,agent2):
        Group=copy.deepcopy(Groups)
        M_set_split=copy.deepcopy(Group.M_set_split)
        M_set_split[agent1].remove(node)
        M_set_split[agent2].append(node)
        Group.M_set_split=M_set_split
        for i in [agent1,agent2]:
            D=np.zeros((len(M_set_split[i]),len(M_set_split[i])))
            id=M_set_split[i]
            for a in range(len(id)):
                for b in range(len(id)):
                    if a!=b:
                        D[a][b]=self.M_fre[id[b]]*self.M_dis[id[a]][id[b]]
            dis=sorted([(sum(D[i][:]),i) for i in range(len(D))])
            if len(id)==0:
                Group.to_t[i]=0
                Group.Center[i]=None
            else:
                Group.to_t[i]=dis[0][0]+sum([self.M_fre[id[k]]*self.To for k in range(len(id))])
                ind=dis[0][1]
                Group.Center[i]=id[ind]
        #print(f" transfer {node} from {agent1} to {agent2} change to {Group.to_t}")
        return Group
    def swap(self,Groups,node1,node2,agent1,agent2):
        Group=copy.deepcopy(Groups)
        M_set_split=copy.deepcopy(Group.M_set_split)
#         Center=copy.deepcopy(Group.Center)
#         to_t=copy.deepcopy(Group.to_t)
        M_set_split[agent1].remove(node1)
        M_set_split[agent1].append(node2)
        M_set_split[agent2].remove(node2)
        M_set_split[agent1].append(node1)
        Group.M_set_split=M_set_split
        for i in [agent1,agent2]:
            D=np.zeros((len(M_set_split[i]),len(M_set_split[i])))
            id=M_set_split[i]
            for a in range(len(id)):
                for b in range(len(id)):
                    if a!=b:
                        D[a][b]=self.M_fre[id[b]]*self.M_dis[id[a]][id[b]]
            dis=sorted([(sum(D[i][:]),i) for i in range(len(D))])
            if len(dis)>0:
                Group.to_t[i]=dis[0][0]+sum([self.M_fre[id[k]]*self.To for k in range(len(id))])
                ind=dis[0][1]
                Group.Center[i]=id[ind]
            else:
                Group.to_t[i]=0
                Group.Center[i]=None
       # print(f" swap {node1} with {node2} from {agent1} to {agent2} change to {Group.to_t}")
        return Group
    def addNode(self,Groups,node,agent):
        Group=copy.deepcopy(Groups)
        M_set_split=copy.deepcopy(Group.M_set_split)
        i=agent
        M_set_split[i].append(node)
        Group.M_set_split=M_set_split
        D=np.zeros((len(M_set_split[i]),len(M_set_split[i])))
        id=M_set_split[i]
        for a in range(len(id)):
            for b in range(len(id)): 
                if a!=b:
                    D[a][b]=self.M_fre[id[b]]*self.M_dis[id[a]][id[b]]
        dis=sorted([(sum(D[i][:]),i) for i in range(len(D))])
        Group.to_t[i]=dis[0][0]+sum([self.M_fre[id[k]]*self.To for k in range(len(id))])
        ind=dis[0][1]
        Group.Center[i]=id[ind]
        #print(f"center with {agent} {len(D)} { Group.to_t[i]}",id[ind])
        return Group
    
    def Add_M(self,add_set,M_fre):
        self.M_fre=M_fre
        Group=copy.deepcopy(self.Groups)
        M_set_split=copy.deepcopy(Group.M_set_split)
        for node in add_set:
            Min_tt=100000000000000
            Best_group=None
            for i in range(self.Drone_num):
                new_group=self.addNode(Group, node, i)
                max_tt=max(new_group.to_t)
                if max_tt<Min_tt:
                    Best_group=new_group
                    #print(f"put {node} to {i} with {max_tt}")
                    Min_tt=max_tt
            Group=Best_group
        self.Groups=Group
        return Group
    
    def Remove_M(self,Remove_set,M_fre):
        self.M_fre=M_fre
#         Group=copy.deepcopy(self.Groups)
#         M_set_split=copy.deepcopy(Group.M_set_split)
        Group=copy.deepcopy(self.Groups)
        M_set_split=copy.deepcopy(Group.M_set_split)
#         Center=copy.deepcopy(Group.Center)
#         to_t=copy.deepcopy(Group.to_t)
        change=[]
        for i in Remove_set:
            for j in range(self.Drone_num):
                if i in M_set_split[j]:
                    M_set_split[j].remove(i)
                    change.append(j)
        Group.M_set_split=M_set_split
        for i in change:
            D=np.zeros((len(M_set_split[i]),len(M_set_split[i])))
            id=M_set_split[i]
            for a in range(len(id)):
                for b in range(len(id)):
                    if a!=b:
                        D[a][b]=self.M_fre[id[b]]*self.M_dis[id[a]][id[b]]
            dis=sorted([(sum(D[i][:]),i) for i in range(len(D))])
            Group.to_t[i]=dis[0][0]+sum([self.M_fre[id[k]]*self.To for k in range(len(id))])
            ind=dis[0][1]
            Group.Center[i]=id[ind]
        self.Groups=Group
        return Group
    
    
    def draw (self,M_set_split,Center):
        co=['y','g','orange','black','b']
        fig=plt.figure()
        ax=fig.add_subplot(111,projection='3d')
    #             ax.scatter([M_loc[i][0] for i in range(len(M_loc))],[M_loc[i][1] for i in range(len(M_loc))],
    #                        [M_loc[i][2] for i in range(len(M_loc))])
        for i in range(len(M_set_split)):
           #print(i)
            ax.scatter([self.dic_M_W.get(i)[0] for i in M_set_split[i]],[self.dic_M_W.get(i)[1] for i in M_set_split[i]],[self.dic_M_W.get(i)[2] for i in M_set_split[i]],color=co[i])
        ax.scatter([self.dic_M_W.get(i)[0] for i in Center],[self.dic_M_W.get(i)[1] for i in Center],[self.dic_M_W.get(i)[2] for i in Center],color='r',s=50)
        plt.show()               
    def do_improve(self):
        t_start=time.time()
        Best_p=copy.deepcopy(self.Groups)
        M_set_split=copy.deepcopy(Best_p.M_set_split)
        Cur_to_t=sorted((Best_p.to_t[i],i) for i in range(len(M_set_split)))
        Max_t,poor=sorted((Best_p.to_t[i],i) for i in range(len(M_set_split)))[-1]
        print(Max_t,poor,Cur_to_t)
        print(f"check first {Best_p.id_set }")
        num_iter=0
        def stop_condition():
            return num_iter>self.Max_iter or time.time()-t_start>self.Max_time
        while not stop_condition():
            print(f"---------- iteration {num_iter}---------\n ")
            print(Best_p.to_t )
            Sum_t=sum(Best_p.to_t)
            Max_t=max(Best_p.to_t)
            print(f" the Max_t {Max_t},{Sum_t}")
            print(f"poor {poor}")
            agents_re=list(set(range(len(Best_p.to_t)))-set([poor]))
    #         while not stop:
            max_t=Max_t
            sum_t=Sum_t
            best_neigh=[]
           # print(f"where is leaf {leafs}, {agents_re}")
            #for i in Best_p.M_set_split[poor]:
            check=200;seq=0;breaker=False
            for i in range(len(Best_p.to_t)-1):
                for j in range(i+1,len(Best_p.to_t)):
                    for a in Best_p.M_set_split[i]:
#                 for j in agents_re:
                        new_group=self.transfer(Best_p,a, i,j)
                    #    print(f"Best_p {Best_p[0].id_set} {j} {i}")
                        max_tt=max(new_group.to_t)
                        sum_tt=sum(new_group.to_t)
                        #print("see here",max_tt,sum_tt,max_t,sum_t)
                     #   print(f" transfer leaf {i} from {poor[0]} to {j} is {max_tt}")
                       # print([i.to_t for i in new_agent])
                         #   max([a1.to_t,a2.to_t]+[Best_p[a].to_t for a in list(set(agents_re)-set([j]))])
                        if max_tt<max_t: #and sum_tt<=sum_t*1.5:
                            max_t=max_tt
                            sum_t=sum_tt
                            best_neigh=([i],j,new_group)
                            print(f" transfer leaf {a} from {i} to {j} is {max_tt}")
                        else:
                            if max_tt<=max_t and sum_tt<sum_t:
                                sum_t=sum_tt
                                best_neigh=([i],j,new_group)
                                print(f" sum changed!: transfer leaf {a} from {i} to {j} is {max_tt}")
                        seq=seq+1
                        if seq>=check:
                            if best_neigh==[]:
                                check=check+200
                                #print(f"cannot break {seq} {check}")
                            else:
                                breaker=True
                                #print(f"here we want to break {seq} {check}")
                                break
                    if breaker==True:
                            break 
                if breaker==True: 
                    break
            print(f"here is best neigh {best_neigh}")
            if best_neigh==[]:
                print(f"get into process other swap")
                sort=list(range(len(Best_p.to_t)))
                random.shuffle(sort)
                check=100;seq=0;breaker=False
                #print(f"check sort ",sort)
                for iii in range(len(Best_p.to_t)-1):
                    for jjj in range(iii+1,len(Best_p.to_t)):
                        i=sort[iii]
                        j=sort[jjj]
                        for a in Best_p.M_set_split[i]:
                            for b in Best_p.M_set_split[j]:
                                new_group=self.swap(Best_p,a,b,i,j)
                            #    print(f"Best_p {Best_p[0].id_set} {j} {i}")
                                max_tt=max(new_group.to_t)
                                sum_tt=sum(new_group.to_t)
                                #print("see here",max_tt,sum_tt,max_t,sum_t)
                             #   print(f" transfer leaf {i} from {poor[0]} to {j} is {max_tt}")
                               # print([i.to_t for i in new_agent])
                                 #   max([a1.to_t,a2.to_t]+[Best_p[a].to_t for a in list(set(agents_re)-set([j]))])
                                #print(f" swap  {a} and {b} from {i} to {j} is {max_tt} {max_t}")
                                if max_tt<max_t:#and sum_tt<=sum_t*1.5:
                                    max_t=max_tt
                                    sum_t=sum_tt
                                    best_neigh=([i],j,new_group)
                                    print(f" swap  {a} and {b} from {i} to {j} is {max_tt}")
                                else:
                                    if max_tt<=max_t and sum_tt<sum_t:
                                        sum_t=sum_tt
                                        best_neigh=([i],j,new_group)
                                        print(f" sum changed!: swap leaf {a}, {b} from {i} to {j} is {max_tt}")
                                seq=seq+1
                                if seq>=check:
                                    if best_neigh==[] and check<=2500:
                                        check=check+100
                                        #print(f"cannot break {seq} {check}")
                                    else:
                                        breaker=True
                                        #print(f"here we want to break {seq} {check}")
                                        break
                            if breaker==True:
                                    break 
                        if breaker==True: 
                            break
                    if breaker==True: 
                            break
                        
#                                 else:
#                                     #if sum_tt<sum_t:
#                                     if max_tt<=max_t and sum_tt<sum_t:
#                                         sum_t=sum_tt
#                                         best_neigh=([i],j,new_group)
#                                         print(f" sum changed!: swap leaf {a}, {b} from {i} to {j} is {max_tt}")
            print(f"here is best neigh {best_neigh}")  
            if best_neigh!=[]: 
                Best_p=best_neigh[-1]
                #self.draw(Best_p.M_set_split,Best_p.Center)
                print(Best_p.M_set_split,Best_p.Center)
                num_iter=num_iter+1
            else: 
                break
        self.Groups=Best_p
        return Best_p
    
   
        
class Auction():
    def __init__(self, Agents,dic_M_W):
        self.Agents=Agents
        self.max_t=max([i.to_t for i in Agents])
        self.dic_M_W=dic_M_W
    def draw (self,M_set_split):
        co=['y','g','orange','black','b']
        fig=plt.figure()
        ax=fig.add_subplot(111,projection='3d')
    #             ax.scatter([M_loc[i][0] for i in range(len(M_loc))],[M_loc[i][1] for i in range(len(M_loc))],
    #                        [M_loc[i][2] for i in range(len(M_loc))])
        for i in range(len(M_set_split)):
           #print(i)
            ax.scatter([self.dic_M_W.get(i)[0] for i in M_set_split[i]],[self.dic_M_W.get(i)[1] for i in M_set_split[i]],[self.dic_M_W.get(i)[2] for i in M_set_split[i]],color=co[i])
        #ax.scatter([dic_M_W.get(i)[0] for i in Center],[dic_M_W.get(i)[1] for i in Center],[dic_M_W.get(i)[2] for i in Center],color='r',s=50)
        plt.show()               
    def transfer(self,Agents,nodes,agent1,agent2): 
        a=copy.deepcopy(Agents)
        if nodes[0] in a[agent1].leaf:
            a[agent1].removeNode(nodes)
        else:
            a[agent1].removeParent(nodes)
        a[agent2].addNode(nodes)
        return a
    def swap(self,Agents,node1,node2,agent1,agent2):
#         a1=copy.deepcopy(self.Agents[agent1])
#         a2=copy.deepcopy(self.Agents[agent1])
        a=copy.deepcopy(Agents)
        #a[agent1].removeNode(node1)
        if node1[0] in a[agent1].leaf:
            a[agent1].removeNode(node1)
        else:
            a[agent1].removeParent(node1)
        a[agent1].addNode(node2)
        #a[agent2].removeNode(node2)
        if node2[0] in a[agent2].leaf:
            a[agent2].removeNode(node2)
        else:
            a[agent2].removeParent(node2)
        a[agent2].addNode(node1)
        return a
    def getMax_t(self):
        return max([i.to_t for i in self.Agents])
    
    def do_auction(self):
        c_Agents=copy.deepcopy(self.Agents)
        Best_p=c_Agents[:]
        print(f"check first {[Best_p[i].id_set for i in range(len(Best_p))]}")
        stop=False
        num_iter=0
        def stop_condition():
            return num_iter>100
            
        
        while not stop_condition():
            print(f"---------- iteration {num_iter}---------\n ")
            print([i.to_t for i in Best_p])
            Max_t=max([i.to_t for i in Best_p])
            Sum_t=sum([i.to_t for i in Best_p])
            print(f" the Max_t {Max_t}")
            poor=[i for i in range(len(Best_p))if Best_p[i].to_t==Max_t]
            print(f"poor {poor}")
            buyyer=Best_p[poor[0]]
            agents_re=list(set(range(len(Best_p)))-set(poor))
    #         while not stop:
            leafs=buyyer.leaf
            max_t=Max_t
            sum_t=Sum_t
            best_neigh=[]
           # print(f"where is leaf {leafs}, {agents_re}")
            for i in leafs:
                for j in agents_re:
                    new_agent=self.transfer(Best_p,[i], poor[0],j)
                #    print(f"Best_p {Best_p[0].id_set} {j} {i}")
                    max_tt=max([i.to_t for i in new_agent])
                    sum_tt=sum([i.to_t for i in new_agent])
                    #print("see here",max_tt,sum_tt,max_t,sum_t)
                 #   print(f" transfer leaf {i} from {poor[0]} to {j} is {max_tt}")
                   # print([i.to_t for i in new_agent])
                     #   max([a1.to_t,a2.to_t]+[Best_p[a].to_t for a in list(set(agents_re)-set([j]))])
                    if max_tt<max_t:
                        max_t=max_tt
                        best_neigh=([i],j,new_agent)
                      #  print(f" transfer leaf {i} from {poor[0]} to {j} is {max_tt}")
     
                    else:
                        #if sum_tt<sum_t:
                        if max_tt==max_t and sum_tt<sum_t:
                            sum_t=sum_tt
                            best_neigh=([i],j,new_agent)
                            print(f" sum changed!: transfer leaf {i} from {poor[0]} to {j} is {max_tt}")
            print(f"here is best neigh {best_neigh}")
            if best_neigh==[]: #  do some swap:
                print("get into the swap process")
                for i in leafs:
                    for j in agents_re:
                        for k in Best_p[j].leaf:
                            new_agent=self.swap(Best_p,[i],[k],poor[0],j)
                            max_tt=max([i.to_t for i in new_agent])
                            sum_tt=sum([i.to_t for i in new_agent])
                            if max_tt<max_t:
                                max_t=max_tt
                                best_neigh=([i],j,new_agent)
                                print(f" swap leaf {i} and {k} from {poor[0]} to {j} is {max_tt}")
     
                            else:
                                if max_tt==max_t and sum_tt<sum_t:
                                    sum_t=sum_tt
                                    best_neigh=([i],j,new_agent)
                                    print(f" sum changed!: swap leaf {i} and {k} from {poor[0]} to {j} is {sum_tt}")
            #############################
#             if best_neigh==[]:
#                 print("get into transfer parent nodes")
#                 parents=list(set(buyyer.id_set)-set(leafs))
#                 for i in parents:
#                     for j in agents_re:
#                         new_agent=self.transfer(Best_p,[i], poor[0],j)
#                     #    print(f"Best_p {Best_p[0].id_set} {j} {i}")
#                         max_tt=max([i.to_t for i in new_agent])
#                         sum_tt=sum([i.to_t for i in new_agent])
#                         #print("see here",max_tt,sum_tt,max_t,sum_t)
#                      #   print(f" transfer leaf {i} from {poor[0]} to {j} is {max_tt}")
#                        # print([i.to_t for i in new_agent])
#                          #   max([a1.to_t,a2.to_t]+[Best_p[a].to_t for a in list(set(agents_re)-set([j]))])
#                         if max_tt<max_t:
#                             max_t=max_tt
#                             best_neigh=([i],j,new_agent)
#                             print(f" transfer parent {i} from {poor[0]} to {j} is {max_tt}")
#          
#                         else:
#                             #if sum_tt<sum_t:
#                             if max_tt==max_t and sum_tt<sum_t:
#                                 sum_t=sum_tt
#                                 best_neigh=([i],j,new_agent)
#                                 print(f" sum changed!: transfer parent {i} from {poor[0]} to {j} is {max_tt}")
#                                 
            ###################################### 
            if best_neigh==[]:
                print("get into the other swap process!!! ")
                for i in agents_re:
                    other=list(set(agents_re)-set([i]))
                    for j in other:
                        for ii in Best_p[i].leaf:
                            for k in Best_p[j].leaf:
                                new_agent=self.swap(Best_p,[ii],[k],i,j)
                                max_tt=max([i.to_t for i in new_agent])
                                sum_tt=sum([i.to_t for i in new_agent])
                                if max_tt<max_t:
                                    max_t=max_tt
                                    best_neigh=([i],j,new_agent)
                                    print(f" other swap !!! leaf {i} and {k} from {poor[0]} to {j} is {max_tt}")
         
                                else:
                                    if max_tt==max_t and sum_tt<sum_t:
                                        sum_t=sum_tt
                                        best_neigh=([i],j,new_agent)
                                        print(f" other swap !!! sum changed!: swap leaf {i} and {k} from {poor[0]} to {j} is {sum_tt}")           
           ########################################
            if best_neigh==[]:
                print("get into other transfer")
                for i in agents_re:
                    other=list(set(agents_re)-set([i]))
                    for j in other:
                        for ii in Best_p[i].leaf:
                #            for k in Best_p[j].leaf:
                            new_agent=self.transfer(Best_p,[ii],i,j)
                            max_tt=max([i.to_t for i in new_agent])
                            sum_tt=sum([i.to_t for i in new_agent])
                            if max_tt<max_t:
                                max_t=max_tt
                                best_neigh=([i],j,new_agent)
                                print(f" other transfer !!! leaf {i}  from {poor[0]} to {j} is {max_tt}")
                            else:
                                if max_tt==max_t and sum_tt<sum_t:
                                    sum_t=sum_tt
                                    best_neigh=([i],j,new_agent)
                                    print(f" other transfer !!! sum changed!: swap leaf {i}  from {poor[0]} to {j} is {sum_tt}")        
            
#             if best_neigh==[]: #  do some swap:
#                 print("get into the swap process")
#                 for i in leafs:
#                     for j in agents_re:
#                         for k in Best_p[j].leaf:
#                             new_agent=self.swap(Best_p,[i],[k],poor[0],j)
#                             max_tt=max([i.to_t for i in new_agent])
#                             sum_tt=sum([i.to_t for i in new_agent])
#                             if max_tt<max_t:
#                                 max_t=max_tt
#                                 best_neigh=([i],j,new_agent)
#                                 print(f" swap leaf {i} and {k} from {poor[0]} to {j} is {max_tt}")
#      
#                             else:
#                                 if max_tt==max_t and sum_tt<sum_t:
#                                     sum_t=sum_tt
#                                     best_neigh=([i],j,new_agent)
#                                     print(f" sum changed!: swap leaf {i} and {k} from {poor[0]} to {j} is {sum_tt}")
#             if best_neigh==[]:
#                 print("get into the parent swap process!!! ")
#                 for i in list(set(buyyer.id_set)-set(leafs)):
#                     for j in agents_re:
#                         for k in Best_p[j].leaf:
#                             new_agent=self.swap(Best_p,[i],[k],poor[0],j)
#                             max_tt=max([i.to_t for i in new_agent])
#                             sum_tt=sum([i.to_t for i in new_agent])
#                             if max_tt<max_t:
#                                 max_t=max_tt
#                                 best_neigh=([i],j,new_agent)
#                                 print(f" swap parent {i} and {k} from {poor[0]} to {j} is {max_tt}")
#      
#                             else:
#                                 if max_tt==max_t and sum_tt<sum_t:
#                                     sum_t=sum_tt
#                                     best_neigh=([i],j,new_agent)
#                                     print(f" sum changed!: swap parent {i} and {k} from {poor[0]} to {j} is {sum_tt}")
#             
            
            #if max_t<Max_t:
               # print(f"{self.Agents[poor[0]].to_t},{self.Agents[poor[0]].id_set}")
           
#             if best_neigh==[]:
#                 print("get into the other Transfer process!!! ")
#                 for i in agents_re:
#                     other=list(set(agents_re)-set([i]))
#                     for j in other:
#                         for ii in list(set(Best_p[i].id_set)-set(Best_p[i].leaf)):
#                             for k in list(set(Best_p[j].id_set)-set(Best_p[j].leaf)):
#                                 new_agent=self.swap(Best_p,[ii],[k],i,j)
#                                 max_tt=max([i.to_t for i in new_agent])
#                                 sum_tt=sum([i.to_t for i in new_agent])
#                                 if max_tt<max_t:
#                                     max_t=max_tt
#                                     best_neigh=([i],j,new_agent)
#                                     print(f" other parent swap !!! leaf {i} and {k} from {poor[0]} to {j} is {max_tt}")
#          
#                                 else:
#                                     if max_tt==max_t and sum_tt<sum_t:
#                                         sum_t=sum_tt
#                                         best_neigh=([i],j,new_agent)
#                                         print(f" other parent swap !!! sum changed!: swap leaf {i} and {k} from {poor[0]} to {j} is {sum_tt}")

            
            
            
            if best_neigh!=[]:    
                Best_p=best_neigh[-1]
                new_set=[Best_p[i].id_set for i in range(len(Best_p))]
                self.draw(new_set)
            else:
                break
            #print(f"check second {[Best_p[i].id_set for i in range(len(Best_p))]}")
            num_iter=num_iter+1
            print(max([i.to_t for i in Best_p]))
        return Best_p
    
    
    
# class Graph: 

#     def __init__(self,vertices,graph):
#         self.V= vertices
#         self.graph = graph
#     def addEdge(self,u,v,w):
#         self.graph.append([u,v,w])
#     def find(self, parent, i):
#         if parent[i] == i:
#             return i
#         return self.find(parent, parent[i])
#     def union(self, parent, rank, x, y):
#         xroot = self.find(parent, x)
#         yroot = self.find(parent, y)
#         if rank[xroot] < rank[yroot]:
#             parent[xroot] = yroot
#             
#             print(f" wow {x, y, y}")
#         elif rank[xroot] > rank[yroot]:
#             parent[yroot] = xroot
#             
#             print(f" wow {x, y, x}")
#         else :
#             parent[yroot] = xroot
#             
#             print(f" wow {x, y, x}")
#             rank[xroot] += 1
#         print(f"see {parent}")
#     def KruskalMST(self):
#         result =[]
#         i,e = 0,0 
#         self.graph = sorted(self.graph,key=lambda item: item[2])
#         parent = [] ; rank = []
#         for node in range(self.V):
#             parent.append(node)
#             rank.append(0) 
#             
#         print(f"see {parent}, {rank}")
#         while e < self.V -1 :
#             u,v,w =  self.graph[i]
#             i = i + 1
#             x = self.find(parent, u)
#             y = self.find(parent ,v)
#             if x != y:
#                 e = e + 1  
#                 result.append([u,v,w])
#                 self.union(parent, rank, x, y)      
#         print(parent)    
#         print("Constructed MST :")
#         print("Vertex A    Vertex B  Weight")
#         for u,v,weight  in result:
#             print ("    %d          %d        %d" % (u,v,weight))
#             
#             
                
    
    #     def update_para(self): 
# #         print ("Edge \tWeight")
# #         summ=0
# #         for i in range(1, self.V): 
# #             summ=summ+self.graph[i][ parent[i] ]
# #             print (self.id_set[parent[i]], "-", self.id_set[i], "\t", self.graph[i][ parent[i] ] )
# #             self.MT.append([parent[i],i,self.graph[i][ parent[i] ]])
#         e=list(itertools.chain.from_iterable([i[:-1] for i in self.MT]))
#         tm= [i for i in e if e.count(i)==1]
#         leaf=[self.id_set[i] for i in tm]
#         print(f"www {self.MT}")
#      #   print(f"sum is {summ}, leaf is {leaf}")
#         self.leaf=leaf
#         #self.length=summ
#         self.to_t=self.to_t+self.length 
#     # A utility function to find the vertex with  
#     # minimum distance value, from the set of vertices  
#     # not yet included in shortest path tree 
graph = [ [0, 2, 0, 6, 0], 
            [2, 0, 3, 8, 5], 
            [0, 3, 0, 0, 7], 
            [6, 8, 0, 0, 9], 
            [0, 5, 7, 9, 0]] 
# g = Agent(0,[2,3,1,5,4],graph) 
# g.primMST()
# print(g.MT)
# 
# g.addNode([])



######################################################  Kruskal Minimum Spanning Tree Algorithm

