'''
Created on May 5, 2020

@author: fangqiliu
'''

import numpy as np
import matplotlib.pyplot as plt 
import scipy.integrate
from numpy import exp
from collections import defaultdict
import re
import math
import copy
T=[600,1200,1500,1800,2100,2400,2700,3000]
h1=[[] for i in range(len(T))]; h_min=[[] for i in range(len(T))];h_max=[[] for i in range(len(T))]
h2=[[] for i in range(len(T))]
ru=[[] for i in range(len(T))]
conf_co=1.645
#filename=['result/com_sum_4.eps','com_loss_tim','run_time' ]
policy=0
Num_result=[]
size=2
for drone_num in [3]:
# for i in range(len(T)):
    Par={};P_w_min={};P_min={};P_h_ac={};P_h_min={}
    for size in [1]:
        for p in [0]:
            f=open(f"./result/dy6_{drone_num}_{size}_{policy}_{p}.txt",'r')
            print(f"./result/dy6_{drone_num}_{size}_{policy}_{p}.txt")
            par='miss'
            track={}
            Result_min={}
            Result_ac={}
            Result_high={}
            Result_miss={}
            Result_w_min={}
            Result_w_sum={}
            Result_sam_av={}
            Result_sam_h_av={}
            Result_sam_min_h={}
            Result_sam_min_ac={}
            Result_sam_d_low={}
            Result_sam_d_h={}
            Result_w_sam_m={}
            Result_w_de_m={}
            for k in range(2):
                Result_min[k]={}
                Result_ac[k]={}
                Result_high[k]={}
                fre_dic={}
                for t in T: 
                    fre_dic[t]={}
                    tmp={}
                    for i in range(9):
                        tmp[i]=[]
                    fre_dic[t]=tmp
                Result_min[k]=copy.deepcopy(fre_dic)
                Result_ac[k]=copy.deepcopy(fre_dic)
                Result_high[k]=copy.deepcopy(fre_dic)
                Result_miss[k]=copy.deepcopy(fre_dic)
                Result_w_min[k]=copy.deepcopy(fre_dic)
                Result_w_sum[k]=copy.deepcopy(fre_dic)
                Result_w_sam_m[k]=copy.deepcopy(fre_dic)
                Result_w_de_m[k]=copy.deepcopy(fre_dic)
                track[k]=copy.deepcopy(fre_dic)
                for r in [Result_sam_av, Result_sam_h_av, Result_sam_min_h,Result_sam_min_ac,Result_sam_d_low, Result_sam_d_h,Result_w_sam_m,Result_w_de_m]:
                    r[k] =copy.deepcopy(fre_dic)
            track_miss=defaultdict(dict)
            for i in ['fire','human','window']:
                track_miss[i]=copy.deepcopy(fre_dic)
            for line in f: 
                #print(line)
                if re.match("random| random",str(line))!=None:
                    #print(line.split(":"))
#                     print(f"match",line)
#                     if line.split(' ')[0]=='':
#                         pass
#                     part=line.split(" ")[0]
#                     print(f"what",part)
#                     fre=int(part.split(":")[-1])
                    num=int(line.split(":")[1][0])
                    print(f"num {num}")
                else:
                    tm=defaultdict(list)
                    if re.match("miss*", str(line))!=None:
                        a=line.split(' ')
                        print(a)
                        track_miss[a[1]][int(a[2])]=[int(a[i]) for i in range(4,len(a)-1)]+[int(a[-1][0])]
                        
            print(track)
            '''
#                         if a[0]=='':
#                             method=int(a[2])
#                             dur=int(a[4])
#                             miss=int(a[5])
#                         else:
#                             method=int(a[1])
#                             dur=int(a[3])
#                             miss=int(a[4])
#                         print(f"here", num,dur,method,miss)
                        #Result_miss[num][dur][method].append(miss)  # min_High
#                     Result_w_min[num][fre][method].append(w_min)
#                     Result_min[num][fre][method].append(min_acc)    # min_ac
#                     Result_ac[num][fre][method].append((av_acc,id,event_size)) # average acc
#                     Result_high[num][fre][method].append((high_sum,id,high_size)) # high_average 
#                     Result_w_sum[num][fre][method].append((w_sum,id,event_size))  # weighted auv
#                     Result_sam_av[num][fre][method].append(sam_av)
#                     Result_sam_h_av[num][fre][method].append(sam_h_av)
#                     Result_sam_min_h[num][fre][method].append(sam_min_h)
#                     Result_sam_min_ac[num][fre][method].append(sam_min_ac)
#                     Result_sam_d_low[num][fre][method].append(sam_de_low)
#                     Result_sam_d_h[num][fre][method].append(sam_de_h)
#                     Result_w_sam_m[num][fre][method].append(w_sam_m)


                        Result_w_de_m[num][dur][method].append(miss)
                    #Result_miss[num][fre][method].append(w_de_m)
#             c_min=copy.deepcopy(Result_min)
#             c_ac=copy.deepcopy(Result_ac)
#             c_high=copy.deepcopy(Result_high)
            c_miss=copy.deepcopy(Result_miss)
#             c_w_min=copy.deepcopy(Result_w_min)
#             c_w_sum=copy.deepcopy(Result_w_sum)
#             ######################################### new sample
#             c_sam_av=copy.deepcopy(Result_sam_av)
#             c_sam_h_av=copy.deepcopy(Result_sam_h_av) 
#             c_sam_min_h=copy.deepcopy(Result_sam_min_h)
#             c_sam_min_ac=copy.deepcopy(Result_sam_min_ac)
#             c_sam_d_low=copy.deepcopy(Result_sam_d_low)
#             c_sam_d_h=copy.deepcopy(Result_sam_d_h)
#             c_w_sam_m=copy.deepcopy(Result_w_sam_m)
            c_w_de_m=copy.deepcopy(Result_w_de_m)
            for i in Result_min:
                for t in T:
                    for k in range(2):
#                         summ=0;size=0
#                         for s in Result_ac.get(i)[t][k]:
#                             summ=summ+s[0]*s[2]
#                             size=size+s[2]
#                         if size!=0:
#                             av=round(summ/size,2)
#                         else:
#                             av=0
#                         ############# for w_sum
#                         summ=0;size=0
#                         for s in Result_w_sum.get(i)[t][k]:
#                             summ=summ+s[0]
#                             size=size+s[2]
#                         if size!=0:
#                             av_w_sum=round(summ/size,2)
#                         else:
#                             av_w_sum=0
#                         ###############
#                         summ=0;size=0
#                         for s in Result_high.get(i)[t][k]:
#                             summ=summ+s[0]
#                             size=size+s[2]
#                         if size!=0:
#                             av_high=round(summ/size,2)
#                         else:
#                             av_high=None
#                         c_ac[i][t][k]=av
#                         c_w_sum[i][t][k]=av_w_sum
#                         c_high[i][t][k]=av_high
#                         c_min[i][t][k]=np.min(Result_min.get(i)[t][k]) 
#                         min_high_set=[]
#                         for s in Result_miss.get(i)[t][k]:
#                             if s[1]!=0:
#                                 min_high_set.append(s[0])
#                         c_miss[i][t][k]=np.min(min_high_set) 
#                         c_w_min[i][t][k]=(Result_w_min.get(i)[t][k][-1])   
#                         c_sam_av[i][t][k]=np.mean(Result_sam_av[i][t][k])
#                         c_sam_h_av[i][t][k]=np.mean(Result_sam_h_av[i][t][k]) 
#                         c_sam_min_h[i][t][k]=np.mean(Result_sam_min_h[i][t][k])
#                         c_sam_min_ac[i][t][k]=np.mean(Result_sam_min_ac[i][t][k])
#                         c_sam_d_low[i][t][k]=np.mean(Result_sam_d_low[i][t][k])
#                         c_sam_d_h[i][t][k]=np.mean(Result_sam_d_h[i][t][k])
#                         c_w_sam_m[i][t][k]=np.mean(Result_w_sam_m[i][t][k])
                        c_w_de_m[i][t][k]=np.mean(Result_w_de_m[i][t][k])
            h_av=[[] for i in range(len(T))]
            h_min=[[] for i in range(len(T))]
            h_high=[[] for i in range(len(T))]
            h_miss=[[] for i in range(len(T))]
            h_w_min=[[] for i in range(len(T))]
            h_w_sum=[[] for i in range(len(T))]
            h_sam_av=[[] for i in range(len(T))]
            h_sam_h_av=[[] for i in range(len(T))]
            h_sam_min_h=[[] for i in range(len(T))]
            h_sam_min_ac=[[] for i in range(len(T))]
            h_sam_d_low=[[] for i in range(len(T))]
            h_sam_d_h=[[] for i in range(len(T))]
            h_w_sam_m=[[] for i in range(len(T))]
            h_w_de_m=[[] for i in range(len(T))]
            
#             c_input=[c_w_sum,c_w_min, c_miss, c_ac, c_min, c_high, 
#                      c_sam_av, c_sam_h_av,c_sam_min_h, c_sam_min_ac, c_sam_d_low,c_sam_d_h,c_w_sam_m,c_w_de_m]
            c_input=[c_w_de_m]
#             data=[h_w_sum,h_w_min, h_miss, h_av, h_min,  h_high,
#                   h_sam_av, h_sam_h_av,h_sam_min_h, h_sam_min_ac, h_sam_d_low,h_sam_d_h,h_w_sam_m,h_w_de_m]
            data=[h_w_de_m]
            for c in range(len(c_input)):
                for k,s in c_input[c].items():
                    for t in range(len(T)):
                        data[c][t].append(list(s[T[t]].values()))
    ###############################################################################
        #data=[h_w_sum,h_w_min, h_miss, h_av, h_min,  h_high]
        #data=[ h_min,  h_high]
            parp=[];std=[]
            for d in data: 
                ha=[];error=[]
                for i in d:
                    tmp=[];err=[]
                    for k in range(len(i[0])):
                        tm=round(np.mean([i[f][k] for f in range(len(i))]),2)
                        sd=np.std([i[f][k] for f in range(len(i))])
                        er=conf_co*(sd/math.sqrt(len(i[0])))
                        tmp.append(tm)
                        err.append(er)
                    ha.append(tmp)   ################ here
                    error.append(err)
                parp.append(ha)
                std.append(error)
            #print(f"see {parp}")
#             data=[h_w_sum,h_w_min, h_miss, h_av, h_min,  h_high,
#                   h_sam_av, h_sam_h_av,h_sam_min_h, h_sam_min_ac, h_sam_d_low,h_sam_d_h,h_w_sam_m,h_w_de_m]
            title=[' Sum of weighted AUC ', 'Sum of weighted minimum AUC ', #'Running time (s)',
                 'Minimum AUC of high-priority targets ',  'Average AUC ', 
                 'Minimum AUC ', 'Average AUC of high-priority targets', 'Average sampling accuracy', 'Average sampling accuracy of high-priority targets', 
                 'Minimum sampling accuracy of high-priority targets', 'Minimum sampling accuracy', 'Minimum detection ratio',  'Minimum detection ratio of high-priority targets ',  ' Sum of weighted sampling accuracy', ' Sum of weighted detection ratio'  ]   
            title=['Missing event']
            name=['w_sum','w_min', 'min_high', 'Av_sum',  'Min_ac','High_ac', 'Sam_av', 'Sam_h_av','Sam_min_h', 'Sam_min_ac', 'Min_d_low', 'Min_d_h', 'W_sam_m','W_de_m']
            #name=['w_sum','w_min', 'min_high', 'Av_sum',  'Min_ac','High_ac', 'Sam_av', 'Sam_h_av','Sam_min_h', 'Sam_min_ac', 'Min_d_low', 'Min_d_h']
            name=['mismatch']
            methods=['Max_ac','Max_cov+Max_min','Greedy_min','Greedy_TSP','Max_cov+Greedy_min','Max_min','Greedy_grain','Greedy_weight_grain','Greedy_weighted_min','Max_Cov+Greedy_weight_grain','Max_Cov+Greedy_min']
            mark=['-+','-*','-o','-o','-^','-o','-+','-*','-o','-.','-^','-o']
            #show=[7,3,4,6,0]
            show=[7,3,4,6,5]
            show=[7,8,3,4,6,5]
            #show=[7,8,9,10]
            legends=[methods[i] for i in show]
            for pa in range(len(parp)): 
            #for pa in [8]: 
                for i in show:
#                     if pa in [0,2,3,4,5]:
#                 #plt.plot(T,[h1[j][i] for j in range(len(T))],marker=mark[i])
#                 #print([av_ha[j][i] for j in range(len(T))])
#                     #print([error[p][j][i]/T[j] for j in range(len(T))])
#                         plt.errorbar(T,[parp[pa][j][i]/T[j] for j in range(len(T))],yerr=[std[pa][j][i]/T[j] for j in range(len(T))],fmt=f"{mark[i]}",capsize=2)
#                     #plt.plot(T,[min_high_ha[j][i] for j in range(len(T))],marker=mark[i])
#                     #plt.errorbar(T,[h1[j][i] for j in range(len(T))],yerr=[[h_min[j][i] for j in range(len(T))],[h_max[j][i] for j in range(len(T))]], capsize=4, fmt=mark[i],markersize=8,   capthick=2)
#                     else:
                        #print(f"seee", std[pa])
                    plt.errorbar(T,[parp[pa][j][i] for j in range(len(T))],yerr=[std[pa][j][i] for j in range(len(T))],fmt=f"{mark[i]}",capsize=2)
                plt.xlabel('Flight Time (s)')
                plt.ylabel(title[pa])
                #plt.yticks(np.arange(0,max([parp[p][j][i]/T[j] for j in range(len(T))])+0.1 , step=0.1))
                plt.grid( linestyle='--', linewidth=1)
                plt.legend(legends,loc=0,prop={'size': 8},ncol=2)
                filename=(f"./result/12_result/{par}_{name[pa]}_{drone_num}_{size}_{policy}_{p}.eps")
                plt.savefig(filename,bbox_inches = 'tight')  
                plt.show()
                plt.clf()    
#######################################################################   

'''






'''
        
        methods=['Max_ac','Max_cov+Max_min','Greedy_min','Greedy_TSP','Max_cov+Greedy_min','Max_min','Max_cov+Greedy_grain','Max_cov+Greedy_weight_grain','Max_cov+Greedy_min']
        mark=['+','*','o','.','^','o','+','*','o','.','^','o']
        
        show=[7,8,3,4,6,0]
        
        Par[p]=w_sum_ha
        P_w_min[p]=w_min_ha
        P_h_min[p]=min_high_ha
        P_min[p]=min_ha
        P_h_ac[p]=high_ha
    legends=['weighted k-medoids+improved', 'k-medoids','k-medoids+improved']
     
    for k,w_sum_ha in Par.items():
        print(f" see w",w_sum_ha)
        for i in [7]:
        #plt.plot(T,[h1[j][i] for j in range(len(T))],marker=mark[i])
       # print([av_ha[j][i] for j in range(len(T))])
            plt.plot(T,[w_sum_ha[j][i]/T[j] for j in range(len(T))],marker=mark[i])
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
    
    for k,w_min_ha in P_w_min.items():
        for i in [7]:
        #plt.plot(T,[h1[j][i] for j in range(len(T))],marker=mark[i])
       # print([av_ha[j][i] for j in range(len(T))])
            plt.plot(T,[w_min_ha[j][i]/T[j] for j in range(len(T))],marker=mark[i])
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
    
    

    for k,min_high_ha in P_h_min.items():
        print(k)
        for i in [7]:
        #plt.plot(T,[h1[j][i] for j in range(len(T))],marker=mark[i])
       # print([av_ha[j][i] for j in range(len(T))])
            plt.plot(T,[min_high_ha[j][i]/T[j] for j in range(len(T))],marker=mark[i])
            #print([min_high_ha[j][i]/T[j] for j in range(len(T))])
            #plt.plot(T,[min_high_ha[j][i] for j in range(len(T))],marker=mark[i])
            #plt.errorbar(T,[h1[j][i] for j in range(len(T))],yerr=[[h_min[j][i] for j in range(len(T))],[h_max[j][i] for j in range(len(T))]], capsize=4, fmt=mark[i],markersize=8,   capthick=2)
#         legends=[methods[i] for i in show]
    plt.xlabel('Flight Time (s)')
    plt.ylabel('Minimum accumulative accuracy of high-priority tasks ')
    plt.grid( linestyle='--', linewidth=1)
    plt.legend(legends,loc=0,prop={'size': 8},ncol=2)
    filename=(f"./result/new_result/{par}_miss_{drone_num}.eps")
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
#     filename=(f"./result/new_result/{par}_miss_{drone_num}.eps")
#     plt.savefig(filename,bbox_inches = 'tight')  
#     #plt.show()
#     plt.clf() 

    for k,min_ha in P_min.items():
        for i in [7]:
        #plt.plot(T,[h1[j][i] for j in range(len(T))],marker=mark[i])
       # print([av_ha[j][i] for j in range(len(T))])
            plt.plot(T,[min_ha[j][i]/T[j] for j in range(len(T))],marker=mark[i])
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
    

    #mark=['+','*','o','.','^','o']
    for k,high_ha in P_h_ac.items():
        for i in [7]:
        #print(T)
        #plt.plot(T,[h1[j][i] for j in range(len(T))],marker=mark[i])
        #print([high_ha[j][i] for j in range(len(T))])
            plt.plot(T,[high_ha[j][i]/T[j] for j in range(len(T))],marker=mark[i])
        #plt.plot(T,[high_ha[j][i] for j in range(len(T))],marker=mark[i])
        #plt.errorbar(T,[h1[j][i] for j in range(len(T))],yerr=[[h_min[j][i] for j in range(len(T))],[h_max[j][i] for j in range(len(T))]], capsize=4, fmt=mark[i],markersize=8,   capthick=2)
    #legends=['Max_ac','Max_cov','Greedy_min','Greedy_TSP','Max_cov+Greedy_min','Max_min']
    plt.xlabel('Flight Time (s)')
    plt.ylabel('Average accumulative accuracy of high-priority tasks ')
    plt.grid( linestyle='--', linewidth=1)
    plt.legend(legends,loc=0,prop={'size': 8},ncol=2)
    filename=(f"./result/new_result/{par}_High_ac_{drone_num}.eps")
    plt.savefig(filename,bbox_inches = 'tight')  
    #plt.show()
    plt.clf() 

'''
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



