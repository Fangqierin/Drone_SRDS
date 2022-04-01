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
T=[300,600,900,1200,1500]
h1=[[] for i in range(len(T))]; h_min=[[] for i in range(len(T))];h_max=[[] for i in range(len(T))]
h2=[[] for i in range(len(T))]
ru=[[] for i in range(len(T))]
#filename=['result/com_sum_4.eps','com_loss_tim','run_time' ]

Num_result=[]
for drone_num in [2,4]:
# for i in range(len(T)):
    f=open(f"./result/sum7_{drone_num}.txt",'r')
    
    #     k=[0,1,3,4]
        #print(k.index(0))
    #V=[[] for i in range(len(k))]
    Result_min={}
    Result_ac={}
    Result_high={}
    for k in range(5):
        Result_min[k]={}
        Result_ac[k]={}
        Result_high[k]={}
        fre_dic={}
        for t in T: 
            fre_dic[t]={}
            tmp={}
            for i in [0,1,2,3,4,5]:
                tmp[i]=[]
            fre_dic[t]=tmp
        
        Result_min[k]=copy.deepcopy(fre_dic)
        Result_ac[k]=copy.deepcopy(fre_dic)
        Result_high[k]=copy.deepcopy(fre_dic)
    
    for line in f: 
        
        if re.match("random",str(line))!=None:
           # print(line.split(":"))
            fre=int(line.split(":")[-1])
            num=int(line.split(":")[1][0])
            
           # print(fre,num)    
        if re.match("[0|1|2|3] -1 [0|2|1|3|4|5]", str(line))!=None:
            a=line.split(' ')
            #drone.id} {i} {j} {re} {min_re[0]} {min_re[1]} {high_sum} {high_size} {miss} {tt} {ii
            #{drone.id} {i} {j} {re} {min_re[0]} {min_re[1]} {miss} {high_sum} {high_size} {tt} {ii}
            id=int(a[0])
            method=int(a[2])
            av_acc=float(a[3])
            min_acc=float(a[4])
            count=-int(a[5])
            high_sum=float(a[7])
            high_size=int(a[8])
            event_size=int(a[6])
            run_time=float(a[9])
            print(event_size,high_size)
            print(Result_min)
            print(num,fre,method)
            Result_min[num][fre][method].append(min_acc)
#             print(min_acc)
#             print((av_acc,id,event_size))
            Result_ac[num][fre][method].append((av_acc,id,event_size))
            Result_high[num][fre][method].append((high_sum,id,high_size))
        
    
    
    c_min=copy.deepcopy(Result_min)
    c_ac=copy.deepcopy(Result_ac)
    c_high=copy.deepcopy(Result_high)
    
    for i in Result_min:
        sm_min=Result_min.get(i)
        sm_ac=Result_ac.get(i)
        sm_high=Result_high.get(i)
       # print(sample)
        for t in T:
            tm_min=sm_min[t]
            tm_ac=sm_ac[t]
            tm_high=sm_high[t]
            for k in [0,1,2,3,4,5]:
                summ=0;size=0
                #print(tm_ac)
                for s in tm_ac[k]:
                    print(s)
                    summ=summ+s[0]*s[2]
                    size=size+s[2]
                print(k,tm_ac[k])
                print(i,tm_ac)
                av=round(summ/size,2)
                summ=0;size=0
                #print(tm_high)
                for s in tm_high[k]:
                  #  print(s)
                    summ=summ+s[0]
                    size=size+s[2]
                av_high=round(summ/size,2)
                c_ac[i][t][k]=av
                c_high[i][t][k]=av_high
                c_min[i][t][k]=np.min(tm_min[k]) 
                
    h_av=[[] for i in range(len(T))]
    h_min=[[] for i in range(len(T))]
    h_high=[[] for i in range(len(T))]
    for k,s in c_ac.items():
        for t in range(len(T)):
            h_av[t].append(list(s[T[t]].values()))
    for k,s in c_min.items():
        for t in range(len(T)):
            h_min[t].append(list(s[T[t]].values()))
    for k,s in c_high.items():
        for t in range(len(T)):
            h_high[t].append(list(s[T[t]].values()))

    av_ha=[];min_ha=[];high_ha=[]
    for i in h_av:
        tmp=[]
        for k in range(len(i[0])):
            tm=round(np.mean([i[f][k] for f in range(len(i))]),2)
            av=round(tm/(len(i[0])),2)
            tmp.append(av)
        av_ha.append(tmp)
    for i in h_min:
        tmp=[]
        for k in range(len(i[0])):
            tm=round(np.mean([i[f][k] for f in range(len(i))]),2)
            av=round(tm/(len(i[0])),2)
            tmp.append(av)
        min_ha.append(tmp)
    for i in h_high:
        tmp=[]
        for k in range(len(i[0])):
            tm=round(np.mean([i[f][k] for f in range(len(i))]),2)
            av=round(tm/(len(i[0])),2)
            tmp.append(av)
        high_ha.append(tmp)
    
    legends=['Max_Cov+Max_Grain','Max_Grain','Max_Wait','Max_ac','Greedy_TSP','Greedy_Min']
    mark=['+','*','o','.','^','o']
    for i in [0,1,2,3,4,5]:
        #plt.plot(T,[h1[j][i] for j in range(len(T))],marker=mark[i])
        #print(i,[av_ha[j][i] for j in range(len(T))])
        plt.plot(T,[av_ha[j][i]/T[j] for j in range(len(T))],marker=mark[i])
        #plt.plot(T,[av_ha[j][i] for j in range(len(T))],marker=mark[i])
       # plt.errorbar(T,[h1[j][i] for j in range(len(T))],yerr=[[h_min[j][i] for j in range(len(T))],[h_max[j][i] for j in range(len(T))]], capsize=4, fmt=mark[i],markersize=8,   capthick=2)
    #legends=['Max_Cov+Max_Grain','Max_Grain','Max_ac','Greedy_TSP']
    plt.xlabel('Flight Time (s)')
    plt.ylabel('Average accumulative accuracy ')
    plt.grid( linestyle='--', linewidth=1)
    plt.legend(legends,loc=0,prop={'size': 12},ncol=2)
    filename=(f"./result/new_result/2Av_sum2_{drone_num}.eps")
    plt.savefig(filename,bbox_inches = 'tight')  
    plt.show()
    plt.clf()  
    
    #Num_result.append([min_ha[j][5]/T[j] for j in range(len(T))])
    mark=['+','*','o','.','^','o']
    for i in [0,1,2,3,4,5]:
        #plt.plot(T,[h1[j][i] for j in range(len(T))],marker=mark[i])
       # print([av_ha[j][i] for j in range(len(T))])
        plt.plot(T,[min_ha[j][i]/T[j] for j in range(len(T))],marker=mark[i])
        #plt.plot(T,[min_ha[j][i] for j in range(len(T))],marker=mark[i])
        #plt.errorbar(T,[h1[j][i] for j in range(len(T))],yerr=[[h_min[j][i] for j in range(len(T))],[h_max[j][i] for j in range(len(T))]], capsize=4, fmt=mark[i],markersize=8,   capthick=2)
    #legends=['Max_Cov+Max_Grain','Max_Grain','Max_ac','Greedy_TSP']
    plt.xlabel('Flight Time (s)')
    plt.ylabel('Minimum accumulative accuracy ')
    plt.grid( linestyle='--', linewidth=1)
    plt.legend(legends,loc=0,prop={'size': 12},ncol=2)
    filename=(f"./result/new_result/2Min_ac2_{drone_num}.eps")
    plt.savefig(filename,bbox_inches = 'tight')  
    plt.show()
    plt.clf()  
    
    mark=['+','*','o','.','^','o']
    for i in [0,1,2,3,4,5]:
        #plt.plot(T,[h1[j][i] for j in range(len(T))],marker=mark[i])
       # print([av_ha[j][i] for j in range(len(T))])
        plt.plot(T,[high_ha[j][i]/T[j] for j in range(len(T))],marker=mark[i])
        #plt.plot(T,[high_ha[j][i] for j in range(len(T))],marker=mark[i])
        #plt.errorbar(T,[high_ha[j][i] for j in range(len(T))],yerr=[[h_min[j][i] for j in range(len(T))],[h_max[j][i] for j in range(len(T))]], capsize=4, fmt=mark[i],markersize=8,   capthick=2)
    #legends=['Max_Cov+Max_Grain','Max_Grain','Max_Cov+Max_ac ','Max_ac','Greedy_TSP','Greedy_Min']
    plt.xlabel('Flight Time (s)')
    plt.ylabel('Average accumulative accuracy of high-priority tasks ')
    plt.grid( linestyle='--', linewidth=1)
    plt.legend(legends,loc=0,prop={'size': 12},ncol=2)
    filename=(f"./result/new_result/2High_ac_{drone_num}.eps")
    plt.savefig(filename,bbox_inches = 'tight')  
    plt.show()
    plt.clf() 
        
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
#     print(f"to seee", [av_ha[j][2] for j in range(len(T))])
#     Num_result.append([av_ha[j][2]/T[j] for j in range(len(T))])
#     mark=['+','*','o','.']
#     for i in range(4):
#          #plt.plot(T,[h1[j][i] for j in range(len(T))],marker=mark[i])
#         plt.plot(T,[av_ha[j][i]/T[j] for j in range(len(T))],marker=mark[i])
#         # plt.errorbar(T,[h1[j][i] for j in range(len(T))],yerr=[[h_min[j][i] for j in range(len(T))],[h_max[j][i] for j in range(len(T))]], capsize=4, fmt=mark[i],markersize=8,   capthick=2)
#     legends=['Cov','Max_min','Max_ac','Greedy_TSP']
#     plt.xlabel('Flight Time (s)')
#     plt.ylabel('Average accumulative accuracy ')
#     plt.grid( linestyle='--', linewidth=1)
#     plt.legend(legends,loc=0,prop={'size': 12},ncol=2)
#     plt.savefig(filename,bbox_inches = 'tight')
#     #plt.clf()  
#     plt.show()
# Num=[1,2,3,4]
# mark=['+','*','o','.']
# for i in range(4):
#     #plt.plot(T,[h1[j][i] for j in range(len(T))],marker=mark[i])
#     plt.plot(T,Num_result[i],marker=mark[i])
#    # plt.errorbar(T,[h1[j][i] for j in range(len(T))],yerr=[[h_min[j][i] for j in range(len(T))],[h_max[j][i] for j in range(len(T))]], capsize=4, fmt=mark[i],markersize=8,   capthick=2)
# legends=['Dron_num=1','Dron_num=2','Dron_num=3','Dron_num=4']
# plt.xlabel('Flight Time (s)')
# plt.ylabel('Average accumulative accuracy ')
# plt.grid( linestyle='--', linewidth=1)
# plt.legend(legends,loc=0,prop={'size': 12},ncol=2)
# filename=(f"./result/com_av_num.eps")
# plt.savefig(filename,bbox_inches = 'tight')
# plt.show()
# plt.clf()  


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
'''
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



