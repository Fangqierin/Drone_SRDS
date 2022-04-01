'''
Created on May 5, 2020

@author: fangqiliu
'''

import numpy as np
import matplotlib.pyplot as plt 
import scipy.integrate
from numpy import exp
import re
import copy
T=[1200,1800,2400,2700]
h1=[[] for i in range(len(T))]; h_min=[[] for i in range(len(T))];h_max=[[] for i in range(len(T))]
h2=[[] for i in range(len(T))]
ru=[[] for i in range(len(T))]
#filename=['result/com_sum_4.eps','com_loss_tim','run_time' ]

Num_result=[]
for drone_num in [2,4]:
# for i in range(len(T)):
    Par={};P_w_min={};P_min={};P_h_ac={};P_h_min={}
    for p in [1,2]:
        f=open(f"./result/dy_{drone_num}_{p}_0-4.txt",'r')
        par='dy'
    #     k=[0,1,3,4]
        #print(k.index(0))
    #V=[[] for i in range(len(k))]
        Result_min={}
        Result_ac={}
        Result_high={}
        Result_min_high={}
        Result_w_min={}
        Result_w_sum={}
        for k in range(0,4):
            Result_min[k]={}
            Result_ac[k]={}
            Result_high[k]={}
            fre_dic={}
            for t in T: 
                fre_dic[t]={}
                tmp={}
                for i in range(11):
                    tmp[i]=[]
                fre_dic[t]=tmp
            Result_min[k]=copy.deepcopy(fre_dic)
            Result_ac[k]=copy.deepcopy(fre_dic)
            Result_high[k]=copy.deepcopy(fre_dic)
            Result_min_high[k]=copy.deepcopy(fre_dic)
            Result_w_min[k]=copy.deepcopy(fre_dic)
            Result_w_sum[k]=copy.deepcopy(fre_dic)
        for line in f: 
            if re.match("random",str(line))!=None:
               # print(line.split(":"))
                fre=int(line.split(":")[-1])
                num=int(line.split(":")[1][0])
                print(f"num {num}")
               # print(fre,num)    
            if re.match("[0|1|2|3|4|5] -1 [0|1|2|3|4|5|6|7|8|9|10]", str(line))!=None:
                a=line.split(' ')
                #drone.id} {i} {j} {re} {min_re[0]} {min_re[1]} {high_sum} {high_size} {miss} {tt} {ii
                ss="{drone.id} {i} {j} {re} {min_re[0]} {min_re[1]} {high_sum} {high_size} {miss} {min_high} {sum_w_v} {w_min_v} {tt} {ii}"
                id=int(a[0])
                method=int(a[2])
                av_acc=float(a[3])
                min_acc=float(a[4])
                count=-int(a[5])
                high_sum=float(a[6])
                high_size=int(a[7])
                event_size=int(a[8])
                min_high=float(a[9])
                w_sum=float(a[10])
                w_min=float(a[11])
                print(f"line {line}")
    #             run_time=float(a[9])
    #             run_time=float(a[7])
                Result_min_high[num][fre][method].append((min_high,high_size))
                Result_w_min[num][fre][method].append(w_min)
                Result_min[num][fre][method].append(min_acc)
                Result_ac[num][fre][method].append((av_acc,id,event_size))
                Result_high[num][fre][method].append((high_sum,id,high_size))
                Result_w_sum[num][fre][method].append((w_sum,id,event_size))
        c_min=copy.deepcopy(Result_min)
        c_ac=copy.deepcopy(Result_ac)
        c_high=copy.deepcopy(Result_high)
        c_min_high=copy.deepcopy(Result_min_high)
        c_w_min=copy.deepcopy(Result_w_min)
        c_w_sum=copy.deepcopy(Result_w_sum)
        for i in Result_min:
            sm_min=Result_min.get(i)
            sm_ac=Result_ac.get(i)
            sm_high=Result_high.get(i)
            sm_min_high=Result_min_high.get(i)
            sum_w_min=Result_w_min.get(i)
            sum_w_sum=Result_w_sum.get(i)
           # print(sample)
            for t in T:
                tm_min=sm_min[t]
                tm_ac=sm_ac[t]
                tm_high=sm_high[t]
                tm_min_high=sm_min_high[t]
                tm_w_min=sum_w_min[t]
                tm_w_sum=sum_w_sum[t]
                for k in range(8):
                    summ=0;size=0
                    for s in tm_ac[k]:
                        summ=summ+s[0]*s[2]
                        size=size+s[2]
                    if size!=0:
                        av=round(summ/size,2)
                    else:
                        av=0
                    ############# for w_sum
                    summ=0;size=0
                    for s in tm_w_sum[k]:
                        summ=summ+s[0]
                        size=size+s[2]
                    if size!=0:
                        av_w_sum=round(summ/size,2)
                    else:
                        av_w_sum=0
                    ###############
                    summ=0;size=0
                    for s in tm_high[k]:
                        summ=summ+s[0]
                        size=size+s[2]
                    if size!=0:
                        av_high=round(summ/size,2)
                    else:
                        av_high=0
                    c_ac[i][t][k]=av
                    c_w_sum[i][t][k]=av_w_sum
                    c_high[i][t][k]=av_high
                    c_min[i][t][k]=np.min(tm_min[k]) 
                    
                    min_high_set=[]
                    # here is to treat multiple drones: 
                    for s in tm_min_high[k]:
                        if s[1]!=0:
                            min_high_set.append(s[0])
                    print(f" min_High_set {tm_min_high[k]} {min_high_set} {line}")
                    c_min_high[i][t][k]=np.min(min_high_set) 
#                     if k==7:
#                         print(f" check minimum high {p} {k} {t} {np.min(tm_min[k])} {tm_min_high[k]} {c_min_high}")
                 #   print(f" check tm_w_min",tm_w_min[k])
                    c_w_min[i][t][k]=(tm_w_min[k][-1])
                    
        h_av=[[] for i in range(len(T))]
        h_min=[[] for i in range(len(T))]
        h_high=[[] for i in range(len(T))]
        h_min_high=[[] for i in range(len(T))]
        h_w_min=[[] for i in range(len(T))]
        h_w_sum=[[] for i in range(len(T))]
        for k,s in c_w_min.items():
            for t in range(len(T)):
                h_w_min[t].append(list(s[T[t]].values()))
        for k,s in c_ac.items():
            for t in range(len(T)):
                h_av[t].append(list(s[T[t]].values()))
        for k,s in c_w_sum.items():
            for t in range(len(T)):
                h_w_sum[t].append(list(s[T[t]].values()))
        for k,s in c_min.items():
            for t in range(len(T)):
                h_min[t].append(list(s[T[t]].values()))
        
        for k,s in c_min_high.items():
            #print(f"seeee {k} {s}")
            for t in range(len(T)):
                h_min_high[t].append(list(s[T[t]].values()))
        for k,s in c_high.items():
            for t in range(len(T)):
                h_high[t].append(list(s[T[t]].values()))
        #print(c_high)
        #print(f"h_high",h_high)
        av_ha=[];min_ha=[];high_ha=[];min_high_ha=[];w_min_ha=[];w_sum_ha=[]
        for i in h_av:
            tmp=[]
            for k in range(len(i[0])):
                tm=round(np.mean([i[f][k] for f in range(len(i))]),2)
                av=round(tm/(len(i[0])),2)
                tmp.append(tm)
            av_ha.append(tmp)
        for i in h_w_sum:
            tmp=[]
            for k in range(len(i[0])):
                tm=round(np.mean([i[f][k] for f in range(len(i))]),2)
                av=round(tm/(len(i[0])),2)
                tmp.append(tm)
            w_sum_ha.append(tmp)
        for i in h_min:
            tmp=[]
            for k in range(len(i[0])):
                tm=round(np.mean([i[f][k] for f in range(len(i))]),2)
                
                av=round(tm/(len(i[0])),2)
                tmp.append(tm)
            min_ha.append(tmp)
        for i in h_min_high:
            print(f" i" ,i)
            tmp=[]
            for k in range(len(i[0])):
                print(f"see",[i[f][k] for f in range(len(i))])
                tm=round(np.mean([i[f][k] for f in range(len(i))]),2)
                if k==7:
                    print(f"tem {tm}{i[0]}" )
                av=round(tm/(len(i[0])),2)
                tmp.append(tm)
                if k==7:
                    print(tmp)
            min_high_ha.append(tmp)
        for i in h_high:
            tmp=[]
            for k in range(len(i[0])):
                tm=round(np.mean([i[f][k] for f in range(len(i))]),2)
                av=round(tm/(len(i[0])),2)
                tmp.append(tm)
            high_ha.append(tmp)
        for i in h_w_min:
            tmp=[]
            for k in range(len(i[0])):
                tm=round(np.mean([i[f][k] for f in range(len(i))]),2)
                av=round(tm/(len(i[0])),2)
                tmp.append(tm)
            w_min_ha.append(tmp)   ################ here weighted mean 
        #print(f"high_ha",high_ha)
        #print(min_high_ha)
        #print(re)
        # tmp=[[] for i in range(len([0,1,3,4]))]
        # for i in re:
        #     for j in range(len(i)):
        #         tmp[j].append(i[j])
        # h1=re
        #             c=k.index(int(a[1]))
        #             print(a[1],c)
        #             #print(f"{j}, here match {line}")
        #             V[c].append(float(a[2]))
        #     h1[i]=[np.mean(V[i]) for i in range(len(V))]
        #     h_min[i]=[np.min(V[i]) for i in range(len(V))]
        #     h_max[i]=[np.max(V[i]) for i in range(len(V))]
        #              print(V)
                #print(j,k[j],V[j])
        #     for f in range(len(V)):
        #         print(V[f])      
           # print(len(V))  
          
                       # print(line)
        #                 h1[i].append(float(a[2]))
        #                 h2[i].append(-float(a[3]))
        #                 ru[i].append(float(a[4]))
        #print([av_ha[j][2] for j in range(len(T))])
        #Num_result.append([av_ha[j][5]/T[j] for j in range(len(T))])
        
        methods=['Max_ac','Max_cov+Max_min','Greedy_min','Greedy_TSP','Max_cov+Greedy_min','Max_min','Max_cov+Greedy_grain','Max_cov+Greedy_weight_grain','Max_cov+Greedy_min']
        mark=['+','*','o','.','^','o','+','*','o','.','^','o']
        
        show=[7,3,4,6,5]
        
        Par[p]=w_sum_ha
        P_w_min[p]=w_min_ha
        P_h_min[p]=min_high_ha
        P_min[p]=min_ha
        P_h_ac[p]=high_ha
    
    #legends=['weighted k-medoids+improved', 'k-medoids','k-medoids+improved']
    legends=[methods[i] for i in show]
    for k,w_sum_ha in Par.items():
#         print(f" see w",w_sum_ha)
        for i in show:
        #plt.plot(T,[h1[j][i] for j in range(len(T))],marker=mark[i])
       # print([av_ha[j][i] for j in range(len(T))])
            plt.plot(T,[w_sum_ha[j][i] for j in range(len(T))],marker=mark[i])
        #plt.plot(T,[min_high_ha[j][i] for j in range(len(T))],marker=mark[i])
        #plt.errorbar(T,[h1[j][i] for j in range(len(T))],yerr=[[h_min[j][i] for j in range(len(T))],[h_max[j][i] for j in range(len(T))]], capsize=4, fmt=mark[i],markersize=8,   capthick=2)

        plt.xlabel('Flight Time (s)')
        plt.ylabel(' Sum of weighted accumulative accuracies ')
        plt.grid( linestyle='--', linewidth=1)
        plt.legend(legends,loc=0,prop={'size': 8},ncol=2)
        filename=(f"./result/new_result/{par}_w_sum_{drone_num}.eps")
        plt.savefig(filename,bbox_inches = 'tight')  
        plt.show()
        plt.clf() 
        
        
        
        
        
    '''
    
    #for k,w_min_ha in P_w_min.items():
    for i in show:
        #plt.plot(T,[h1[j][i] for j in range(len(T))],marker=mark[i])
       # print([av_ha[j][i] for j in range(len(T))])
            plt.plot(T,[w_min_ha[j][i] for j in range(len(T))],marker=mark[i])
        #plt.plot(T,[min_high_ha[j][i] for j in range(len(T))],marker=mark[i])
        #plt.errorbar(T,[h1[j][i] for j in range(len(T))],yerr=[[h_min[j][i] for j in range(len(T))],[h_max[j][i] for j in range(len(T))]], capsize=4, fmt=mark[i],markersize=8,   capthick=2)
    plt.xlabel('Flight Time (s)')
    plt.ylabel('Sum of weighted minimum accumulative accuracies ')
    plt.grid( linestyle='--', linewidth=1)
    plt.legend(legends,loc=0,prop={'size': 8},ncol=2)
    filename=(f"./result/new_result/{par}_w_min_{drone_num}.eps")
    plt.savefig(filename,bbox_inches = 'tight')  
    plt.show()
    plt.clf()
    
    

    #for k,min_high_ha in P_h_min.items():
    print(k)
    for i in show:
        #plt.plot(T,[h1[j][i] for j in range(len(T))],marker=mark[i])
       # print([av_ha[j][i] for j in range(len(T))])
            plt.plot(T,[min_high_ha[j][i] for j in range(len(T))],marker=mark[i])
            #print([min_high_ha[j][i]/T[j] for j in range(len(T))])
            #plt.plot(T,[min_high_ha[j][i] for j in range(len(T))],marker=mark[i])
            #plt.errorbar(T,[h1[j][i] for j in range(len(T))],yerr=[[h_min[j][i] for j in range(len(T))],[h_max[j][i] for j in range(len(T))]], capsize=4, fmt=mark[i],markersize=8,   capthick=2)
    legends=[methods[i] for i in show]
    plt.xlabel('Flight Time (s)')
    plt.ylabel('Minimum accumulative accuracy of high-priority tasks ')
    plt.grid( linestyle='--', linewidth=1)
    plt.legend(legends,loc=0,prop={'size': 8},ncol=2)
    filename=(f"./result/new_result/{par}_min_high_{drone_num}.eps")
    plt.savefig(filename,bbox_inches = 'tight')  
    plt.show()
    plt.clf() 
    

#     
#     for i in show:
#         #plt.plot(T,[h1[j][i] for j in range(len(T))],marker=mark[i])
#        # print([av_ha[j][i] for j in range(len(T))])
#         plt.plot(T,[av_ha[j][i]/T[j] for j in range(len(T))],marker=mark[i])
#         #plt.plot(T,[av_ha[j][i] for j in range(len(T))],marker=mark[i])
#         #plt.errorbar(T,[h1[j][i] for j in range(len(T))],yerr=[[h_min[j][i] for j in range(len(T))],[h_max[j][i] for j in range(len(T))]], capsize=4, fmt=mark[i],markersize=8,   capthick=2)
#     plt.xlabel('Flight Time (s)')
#     plt.ylabel('Average accumulative accuracy ')
#     plt.grid( linestyle='--', linewidth=1)
#     plt.legend(legends,loc=0,prop={'size': 8},ncol=2)
#     filename=(f"./result/new_result/{par}_Av_sum_{drone_num}.eps")
#     plt.savefig(filename,bbox_inches = 'tight')  
#     plt.show()
#     plt.clf()  
        
#     
#     for k,min_high_ha in P_h_min.items():
#         for i in [7]:
#         #plt.plot(T,[h1[j][i] for j in range(len(T))],marker=mark[i])
#        # print([av_ha[j][i] for j in range(len(T))])
#             plt.plot(T,[min_high_ha[j][i]/T[j] for j in range(len(T))],marker=mark[i])
#         #plt.plot(T,[min_high_ha[j][i] for j in range(len(T))],marker=mark[i])
#         #plt.errorbar(T,[h1[j][i] for j in range(len(T))],yerr=[[h_min[j][i] for j in range(len(T))],[h_max[j][i] for j in range(len(T))]], capsize=4, fmt=mark[i],markersize=8,   capthick=2)
#     
#     plt.xlabel('Flight Time (s)')
#     plt.ylabel('Minimum accumulative accuracy of high-priority tasks ')
#     plt.grid( linestyle='--', linewidth=1)
#     plt.legend(legends,loc=0,prop={'size': 8},ncol=2)
#     filename=(f"./result/new_result/{par}_min_high_{drone_num}.eps")
#     plt.savefig(filename,bbox_inches = 'tight')  
#     #plt.show()
#     plt.clf() 

    #for k,min_ha in P_min.items():
    for i in show:
        #plt.plot(T,[h1[j][i] for j in range(len(T))],marker=mark[i])
       # print([av_ha[j][i] for j in range(len(T))])
            plt.plot(T,[min_ha[j][i] for j in range(len(T))],marker=mark[i])
        #plt.plot(T,[min_ha[j][i] for j in range(len(T))],marker=mark[i])
        #plt.errorbar(T,[h1[j][i] for j in range(len(T))],yerr=[[h_min[j][i] for j in range(len(T))],[h_max[j][i] for j in range(len(T))]], capsize=4, fmt=mark[i],markersize=8,   capthick=2)
    #legends=['Max_ac','Max_cov','Greedy_min','Greedy_TSP','Max_cov+Greedy_min','Max_min']
    plt.xlabel('Flight Time (s)')
    plt.ylabel('Minimum accumulative accuracy ')
    plt.grid( linestyle='--', linewidth=1)
    plt.legend(legends,loc=0,prop={'size': 8},ncol=2)
    filename=(f"./result/new_result/{par}_Min_ac_{drone_num}.eps")
    plt.savefig(filename,bbox_inches = 'tight')  
    #plt.show()
    plt.clf()  
    '''

    #mark=['+','*','o','.','^','o']
    #for k,high_ha in P_h_ac.items():
    for i in show:
        #print(T)
        #plt.plot(T,[h1[j][i] for j in range(len(T))],marker=mark[i])
        #print([high_ha[j][i] for j in range(len(T))])
            plt.plot(T,[high_ha[j][i] for j in range(len(T))],marker=mark[i])
        #plt.plot(T,[high_ha[j][i] for j in range(len(T))],marker=mark[i])
        #plt.errorbar(T,[h1[j][i] for j in range(len(T))],yerr=[[h_min[j][i] for j in range(len(T))],[h_max[j][i] for j in range(len(T))]], capsize=4, fmt=mark[i],markersize=8,   capthick=2)
    #legends=['Max_ac','Max_cov','Greedy_min','Greedy_TSP','Max_cov+Greedy_min','Max_min']
    plt.xlabel('Flight Time (s)')
    plt.ylabel('Average accumulative accuracy of high-priority tasks ')
    plt.grid( linestyle='--', linewidth=1)
    plt.legend(legends,loc=0,prop={'size': 8},ncol=2)
    filename=(f"./result/new_result/{par}_High_ac_{drone_num}.eps")
    plt.savefig(filename,bbox_inches = 'tight')  
    plt.show()
    plt.clf() 


'''


# Num=[2,3,4]
# T=[300,600,900,1200,1500]
# mark=['+','*','o','.']
# for i in range(4):
#     #plt.plot(T,[h1[j][i] for j in range(len(T))],marker=mark[i])
#     plt.plot(T,Num_result[i],marker=mark[i])
#    # plt.errorbar(T,[h1[j][i] for j in range(len(T))],yerr=[[h_min[j][i] for j in range(len(T))],[h_max[j][i] for j in range(len(T))]], capsize=4, fmt=mark[i],markersize=8,   capthick=2)
# legends=['Dron_num=1','Dron_num=2','Dron_num=3','Dron_num=4']
# plt.xlabel('Flight Time (s)')
# plt.ylabel('Minimum accumulative accuracy ')
# plt.grid( linestyle='--', linewidth=1)
# plt.legend(legends,loc=0,prop={'size': 12},ncol=2)
# filename=(f"./result/com_min_num.eps")
# plt.savefig(filename,bbox_inches = 'tight')
# plt.show()
# plt.clf() 

# for drone_num in [2,4,5]: 
#     for i in [0,2]:
#         f=open(f"./result/par2_{drone_num}_{i}.txt",'r')
#         par='show'

'''




'''
for i in range(3):
    plt.plot(T,[h2[j][i] for j in range(len(T))],marker=mark[i])
legends=['h1','h2','h2&Tabu']
plt.xlabel('Flight Time')
plt.ylabel('Time of Loss Monitoring')
plt.grid( linestyle='--', linewidth=1)
plt.legend(legends,loc=0,prop={'size': 12},ncol=2)
plt.savefig(filename[1],bbox_inches = 'tight')
plt.show()

for i in range(3):
    plt.plot(T,[ru[j][i] for j in range(len(T))],marker=mark[i])
legends=['h1','h2','h2&Tabu']
plt.xlabel('Flight Time')
plt.ylabel('Running Time')
plt.grid( linestyle='--', linewidth=1)
plt.legend(legends,loc=0,prop={'size': 12},ncol=2)
plt.savefig(filename[2],bbox_inches = 'tight')
plt.show()

# def f(v,dy,x,f):
#     return (v*((1-dy)**x)) if f==1 else v-(dy*x)
# dy=0.06
# 
# i = scipy.integrate.quad(lambda x: f(0.8,dy,x,1), 0, 4)
# i2=scipy.integrate.quad(lambda x: f(0.8,dy,x,0), 0, 4)
# print (i[0]-i2[0])
# 
# Tl=[(8,1),(15,0.8),(20,1)]
# def f1(Tl,Ti,Time,v_in,dy):
#     k=0
#     t=Ti
#     v=v_in
#     x=[];y=[]
#     off=1
#     last=0
#     ini=0
#     if Tl[-1][0]<Time:
#         Tl=Tl+[(Time,0)]
#     while k<len(Tl): 
#         while t<Tl[k][0]:
#             dur=t-last
#             x.append(t)
#             v=max(ini*pow(1-dy,dur),0)
#             y.append(v)
#             print(t,v,"not tl")
#             t=t+off
#            
#         v=max(Tl[k][1],ini*pow(1-dy,t-last))
#         ini=max(Tl[k][1],ini*pow(1-dy,t-last))
#         y.append(v)
#         x.append(Tl[k][0])
#         print(t,v,"tl")
#         t=Tl[k][0]+off
#         last=Tl[k][0]
#         k=k+1
#         
#     return x,y
# x,y=f1(Tl,0,60,0,0.05)
# # print(x,y)
# # plt.plot(x,y)
# # plt.show()
# l=['1','2','3']
# for i in l:
#     for j in l:
#         pass
#     print("i")

# print("ff",ff)
# print("how about",aa)
# for i in aa:
#     print("see aa",i)
# print("1",aa)
# c=aa
# print("c",c)
# aa[0]=[0]
# print("aa",aa)
# print(c)

# a=a+b
# b.remove(1)
# b.append(33)
# print(a)
# print([i for i in b if i not in a])
'''
'''    
import multiprocessing as mp
#print("Number of processors: ", mp.cpu_count())    
from time import sleep
def calculate(process_name, tasks, results):
    print('[%s] evaluation ' % process_name)
    while True: 
        new_value=tasks.get()
        if new_value<0:
            print('[%s] quit ' % process_name)
            results.put(-1)
            break
        else:
            compute=new_value**2
            sleep(0.02*new_value)
            print('[%s] recived : %i ' % (process_name,new_value))
            results.put(compute)
    return
if __name__=="main":
    manager=mp.Manager()
    tasks=manager.Queue()
    results=manager.Queue()
    num_processes=4
    pool=mp.Pool(processes=num_processes)
    processes=[]
    for i in range(num_processes):

    # Set process name
        process_name = 'P%i' % i

    # Create the process, and connect it to the worker function
        new_process = mp.Process(target=calculate, args=(process_name,tasks,results))

    # Add new process to the list of processes
        processes.append(new_process)

    # Start the process
        new_process.start()    
    
    task_list=[43,1,787]
    for single_task in task_list:
        tasks.put(single_task)
    sleep(5)
    
    
    for i in range(num_processes):
        tasks.put(-1)

# Read calculation results
    num_finished_processes = 0
    while True:
    # Read result
        new_result = results.get()

    # Have a look at the results
        if new_result == -1:
        # Process has finished
            num_finished_processes += 1

        if num_finished_processes == num_processes:
            break
    else:
        # Output result
        print('Result:' + str(new_result))
    
 '''   
# import random
# a=random.choice(range(4))
# print(random.sample(range(5),3))
# import random
# print([random.random()for i in range(10)])



