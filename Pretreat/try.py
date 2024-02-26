# nohup python3 Cover_sum.py 300   >& /tmp/new_sum_300.txt 2>&1  &
# nohup python3 Cover_sum.py 600  >& /tmp/new_re_600.txt 2>&1  &
# nohup python3 Cover_sum.py 900   >& /tmp/new_re_900.txt 2>&1  &
# nohup python3 Cover_sum.py 1200   >& /tmp/new_re_1200.txt 2>&1  &
# nohup python3 Cover_sum.py 1500   >& /tmp/new_re_1500.txt 2>&1  &
# a=[[1,2,3],[2,1,3]]
# b=[[1,0,1],[1,2,1]]
# for i in range(len(a)):
#     print(a[i],b[i])
# import numpy as np
# s1=np.array([1,2,3])
# s2=np.array([1,1,1])
# print(list(s1/s2))
# d={}
# d[1]=[1,2]
# d[2]=[3,4]
# d.get(2)[0]=5
# print(d)
from collections import defaultdict
import numpy as np
class bb:
    def __init__(self,a,b):
        self.a=a
        self.b=b
di={3:bb(1,3),4:bb(2,4)} 
di.pop(3)
aa=[1,2,3]
b=[]
#print([a for a in aa if a in b])
time=list(np.arange(1,10,3))
tl_add=list(zip(time,[0]*len(time))) 
#print(list(map(list, zip(time,[0]*len(time)))))
import matplotlib.pyplot as plt
import numpy as np

import numpy as np
import matplotlib.pyplot as plt
 
a=dict(zip([1,2,3],[1]*3))
b={2:3}
#print(20%10)

a=[1,2,3,'5']

# for i,j in enumerate(a):
#     print(i,j)

a='sssf'
def f(n):
    if n==1:
        return 1
    if n==2:
        return 2
    f1=1;f2=2;
    for k in range(3,n+1):
        tmp=f1+f2
        f1=f2
        f2=tmp
    return tmp
print(f(2))
gold=[300,200,350,400,500]
peopleNeed=[4,7,3,5,5]
#//maxGold[i][j] 保存了i个人挖前j个金矿能够得到的最大金子数，等于 -1 时表示未知

def GetMaxGold(people, mineNum):
    print(people,mineNum)
    if maxGold[people][mineNum]!=-1:
            retMaxGold=maxGold[people][mineNum]
            print(f" we get",retMaxGold,people, mineNum )
    elif mineNum==0:
        print(f"to 0", people, mineNum)
        if people>=peopleNeed[mineNum]:
            retMaxGold=gold[mineNum]
            print(f"see what is the gold",gold[mineNum] )
        else:
            retMaxGold=0
    elif people>=peopleNeed[mineNum]:
        print(f"see",people-peopleNeed[mineNum],mineNum - 1,people,mineNum - 1)
        retMaxGold = max(
                         GetMaxGold(people-peopleNeed[mineNum],mineNum - 1) + gold[mineNum],
                         GetMaxGold(people,mineNum - 1)
                         )
        
        maxGold[people][mineNum] = retMaxGold
    else:
        print(f"not enough ",people, mineNum )
        retMaxGold=GetMaxGold(people,mineNum-1)
        maxGold[people][mineNum] = retMaxGold
    return retMaxGold
people=25
maxGold=[[-1]*(len(gold)+1)]*(people+1)
for i in range(3,2):
    print(i)
# print(maxGold[people])
# print(GetMaxGold(people, len(gold)-1))

# import dash
# import dash_bootstrap_components as dbc
# import dash_core_components as dcc
# import dash_html_components as html
# from dash.dependencies import Input, Output
# import dash_table
# import pandas as pd
# import sqlite3

# import plotly.graph_objs as go
# conn = sqlite3.connect(r"C:\Users\MTGro\Desktop\coding\wineApp\db\wine_data.sqlite")
# c = conn.cursor()
# df = pd.read_sql("select * from wine_data", conn)
# df = df[['country', 'description', 'rating', 'price','province','title','variety','winery','color']]
# df.head(1)

for i in range(4):
    for j in range(i+1,5):
        print(i,j)
a=3
a=a/4+2
print(a)
for i in range(1,4,2):
    print(i)
print(np.arange(0, 40,1))

import copy
class cc():
    def __init__(self,p):
        self.p=p
        self.value=p
def fsss(a):
    c=copy.deepcopy(a)
    c.value+=3
    return c
a=cc(0)
c=fsss(a)
print(a.value,c.value)







# while True: 
#     if a==0:
#         a=2
#     else:
#         if a==2: a=0
#         else:
#             print(f"here")
#             break
#  
# b=[1,2,3]
# b.pop()
# b.pop()
# print(b)
# a=0
# if a:
#     print("a")
# else:
#     print("n")
# a={};b={}
# for i in [a,b]:
#     i[0]=1
#     
# print(a,b)
# a={-0:1}
# a[0]=2
# print(a)
# import math
# import random
# import numpy as np
# print(math.gcd(6, 12))
# a=dict(zip([1],[2]))
# print(a)
# a.pop(1)
# print(a)
# breaker=False
# for i in range(3):
#     for j in range(3):
#         for k in range(3):
#             if k==2:
#                 print(f"to break",i,j,k)
#                 breaker=True
#                 break
#             print(i,j,k)
#         if breaker==True: 
#             break
#     if breaker==True: 
#             break
# a=[20,3];b=[2,4]
# print()
# print([int(i) for i in list(np.array(a)/2) ])
# #a.remove(3)
# a.append(3)
# print([1,2]+[2,3]+[3,4])
# a=list(range(10))
# random.shuffle(a)
# print(a)
# # width of the bars
# barWidth = 0.3
#   
# # Choose the height of the blue bars
# bars1 = [10, 9, 2]
#   
# # Choose the height of the cyan bars
# bars2 = [10.8, 9.5, 4.5]
#   
# # Choose the height of the error bars (bars1)
# yer1 = [0.5, 0.4, 0.5]
#   
# # Choose the height of the error bars (bars2)
# yer2 = [1, 0.7, 1]
#   
# # The x position of bars
# r1 = np.arange(len(bars1))
# r2 = [x + barWidth for x in r1]
#   
# # Create blue bars
# plt.bar(r1, bars1, width = barWidth, color = 'blue', edgecolor = 'black', yerr=yer1, capsize=7, label='poacee')
#   
# # Create cyan bars
# plt.bar(r2, bars2, width = barWidth, color = 'cyan', edgecolor = 'black', yerr=yer2, capsize=7, label='sorgho')
#   
# # general layout
# plt.xticks([r + barWidth for r in range(len(bars1))], ['cond_A', 'cond_B', 'cond_C'])
# plt.ylabel('height')
# plt.legend()
#   
# # Show graphic
# plt.show()
# 
# a=[1,2,3,4]
# 
# print(np.mean(a),(np.std(a))**2)
# 
# b=[i/10 for i in a]
# 
# print(np.mean(b),(np.std(a))**2/(np.std(b))**2)
# print(len('11122'))
# import random 
# random.seed(1)
# print(random.random())
# print([random.random() for i in range(3)])