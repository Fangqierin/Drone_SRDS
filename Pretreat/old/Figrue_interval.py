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
from collections import defaultdict
T=[300,600,900,1200,1500]
h1=[[] for i in range(len(T))]; h_min=[[] for i in range(len(T))];h_max=[[] for i in range(len(T))]
h2=[[] for i in range(len(T))]
ru=[[] for i in range(len(T))]
#filename=['result/com_sum_4.eps','com_loss_tim','run_time' ]

Num_result=[]
for drone_num in [3]:
# for i in range(len(T)):
    #f=open(f"./result/log_interval_{drone_num}.txt",'r')
    f=open(f"./result/log_interval_3.txt",'r')
    #filename=(f"./result/Min_sum_{drone_num}.eps")
    filename=f"./result/visit2_{3}.eps"
    #     k=[0,1,3,4]
        #print(k.index(0))
    #V=[[] for i in range(len(k))]
    Result0=defaultdict(dict)
    Result1=defaultdict(dict)
    tasks=['df','dh', 'mf', 'mw', 'mh']
    methods=[0,1,2,3,4,5]
    for line in f: 
        for i in methods:
            if re.match(f"-1",str(line))!=None:
                method=int(line.split(" ")[1])
                task=str(line.split(" ")[2])
                ty=int(line.split(" ")[3])
                num=float(line.split(" ")[4])
                #num=float(line.split(" ")[5])
                if ty==0:
                    Result0[method][task]=num
                else:
                    Result1[method][task]=num
    print(Result0,Result1)
    c_mean=[[] for i in range(len(methods))]
    f_mean=[[] for i in range(len(methods))]
    seq=0
    for i in methods:
        c_mean=[Result0[i][j] for j in tasks]
        f_mean=[Result1[i][j] for j in tasks]
        print(c_mean,f_mean)
        width = 0.15  
        X=np.arange(len(tasks))
        p1 = plt.bar(X+seq, c_mean, width,color='green')
        p2 = plt.bar(X+seq, f_mean, width, bottom=c_mean,color='blue')
        seq=seq+0.15
#     fig = plt.figure()
#     ax = fig.add_axes([0,0,1,1])
#     X = np.arange(4)
#     ax.bar(X + 0.00, data[0], color = 'b', width = 0.25)
#     ax.bar(X + 0.25, data[1], color = 'g', width = 0.25)
#     ax.bar(X + 0.50, data[2], color = 'r', width = 0.25)
#     N = 5
#     menMeans = (25, 32, 34, 20, 25)
#     womenMeans = (25, 32, 34, 20, 25)
#     menStd = (2, 3, 4, 1, 2)
#     womenStd = (3, 5, 2, 3, 3)
#     ind = np.arange(N)    # the x locations for the groups
#     width = 0.35       # the width of the bars: can also be len(x) sequence
#      
#     p1 = plt.bar(ind, menMeans, width, )
#     p2 = plt.bar(ind, womenMeans, width, bottom=menMeans,)
    label = ['Max_cov', 'Max_cov', 'Greedy_min', 'Greedy_TSP', 'Max_cov+Greedy_min', 'Max_min']
    seq=1
    for i in range(len(label)):
        plt.text(x =5000 , y =150-seq , s = label[i], size = 10)
        seq=seq+1
    plt.ylabel('Visits')
    plt.title(f"Average visiting times of different tasks")
    plt.xticks(range(len(tasks)), ('df','dh', 'mf', 'mw', 'mh'))
    #plt.yticks(np.arange(0, 81, 10))
    plt.legend((p1[0], p2[0]), ('Coarse', 'Fine'))
    
    plt.savefig(filename,bbox_inches = 'tight') 
    plt.show()
#     
    '''
    Result={}
    for k in range(10):
        Result[k]={}
        fre_dic={}
        for t in T: 
            fre_dic[t]={}
            tmp={}
            for i in [0,1,2,3,4,5]:
                tmp[i]=[]
            fre_dic[t]=tmp
        Result[k]=fre_dic
    #print(Result)
    for line in f: 
        if re.match("random",str(line))!=None:
           # print(line.split(":"))
            fre=int(line.split(":")[-1])
            num=int(line.split(":")[1][0])
           # print(fre,num)    
        if re.match("[0|1|2|3|4|5] -1 [0|1|2|3|4|5]", str(line))!=None:
            a=line.split(' ')
            id=int(a[0])
            method=int(a[2])
            av_acc=float(a[3])
            min_acc=float(a[4])
            count=-int(a[5])
            event_size=int(a[6])
            run_time=float(a[7])
           # Result[num][fre][method].append(min_acc)
            Result[num][fre][method].append((av_acc,id,event_size))
    print(Result)
    
    co=copy.deepcopy(Result)
    for i in Result:
        sample=Result.get(i)
       # print(sample)
        for t in T:
            tm=sample[t]
            for k in [0,1,2,3,4,5]:
                sum=0;size=0
                #print(tm[k])
                for s in tm[k]:
                    sum=sum+s[0]*s[2]
                    size=size+s[2]
                av=round(sum/size,2)
             #   co[i][t][k]=np.min(tm[k]) 
                co[i][t][k]=av
    h_av=[[] for i in range(len(T))]
    for k,s in co.items():
       # print(s)
        for t in range(len(T)):
            h_av[t].append(list(s[T[t]].values()))
    ha=copy.deepcopy(h_av)
    #re=[]
    #print(h_av)
    av_ha=[]
    for i in h_av:
        tmp=[]
        for k in range(len(i[0])):
           # print(k)
           # print(([i[f][k] for f in range(len(i))]))
            tm=round(np.mean([i[f][k] for f in range(len(i))]),2)
           # print(tm)
            av=round(tm/(len(i[0])),2)
            tmp.append(av)
        av_ha.append(tmp)
    print(av_ha)
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
    print([av_ha[j][2] for j in range(len(T))])
    Num_result.append([av_ha[j][5]/T[j] for j in range(len(T))])
    mark=['+','*','o','.','^','o']
    for i in [0,1,2,3,4,5]:
        #plt.plot(T,[h1[j][i] for j in range(len(T))],marker=mark[i])
       # print([av_ha[j][i] for j in range(len(T))])
        plt.plot(T,[av_ha[j][i]/T[j] for j in range(len(T))],marker=mark[i])
       # plt.errorbar(T,[h1[j][i] for j in range(len(T))],yerr=[[h_min[j][i] for j in range(len(T))],[h_max[j][i] for j in range(len(T))]], capsize=4, fmt=mark[i],markersize=8,   capthick=2)
    legends=['Max_ac','Max_cov','Greedy_min','Greedy_TSP','Max_cov+Greedy_min','Max_min']
    plt.xlabel('Flight Time (s)')
    plt.ylabel('Minimum accumulative accuracy ')
    plt.grid( linestyle='--', linewidth=1)
    plt.legend(legends,loc=0,prop={'size': 12},ncol=2)
    plt.savefig(filename,bbox_inches = 'tight')  
    plt.show()
    plt.clf()  
Num=[1,2,3,4]
T=[300,600,900,1200,1500]
mark=['+','*','o','.']
for i in range(4):
    #plt.plot(T,[h1[j][i] for j in range(len(T))],marker=mark[i])
    plt.plot(T,Num_result[i],marker=mark[i])
   # plt.errorbar(T,[h1[j][i] for j in range(len(T))],yerr=[[h_min[j][i] for j in range(len(T))],[h_max[j][i] for j in range(len(T))]], capsize=4, fmt=mark[i],markersize=8,   capthick=2)
legends=['Dron_num=1','Dron_num=2','Dron_num=3','Dron_num=4']
plt.xlabel('Flight Time (s)')
plt.ylabel('Minimum accumulative accuracy ')
plt.grid( linestyle='--', linewidth=1)
plt.legend(legends,loc=0,prop={'size': 12},ncol=2)
filename=(f"./result/com_min_num.eps")
plt.savefig(filename,bbox_inches = 'tight')
plt.show()
plt.clf()  
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



