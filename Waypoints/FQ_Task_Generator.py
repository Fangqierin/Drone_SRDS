from random import choice
import threading 
import zmq 
import time
from pyknow import *
from collections import defaultdict
import numpy as np
class DetectFire(Fact):
    pass
class Grid(Fact):
    pass
class Phase2(Fact):
    pass
class Time(Fact):
    pass
class Theta(Fact):
    pass
class Mission(Fact):
    pass
class UpdateEFA(Fact):
    pass
class TaskManager():
    def __init__(self, Missions,theta=3,plantime=20):
        self.TG=TaskGenerator()
        self.TG.reset()
        self.theta=theta
        self.latime=0
        self.plantime=plantime
        self.TG.declare(Theta(v=theta, plan=plantime))
        self.Missions=Missions
        self.Mission_dict=defaultdict 
    def reset(self):
        self.TG.rest()
        tasks=defaultdict(dict)
    def RefineTask(self):
        pass
    def DeclareGridEFA(self, EFAM, init=0):
        rows,columns=EFAM.shape
        self.latime=init
        self.EFA=EFAM
        for x in range(rows):
            for y in range(columns):
                if EFAM[x,y]==0:
                    st='B'
                else:
                    st='N'
                self.TG.declare(Grid(id=(x,y),state=st, EFA=EFAM[x,y],time=init))
        self.TG.run()
        self.cur_tasks=tasks
        return tasks
    def ReportFire(self,Grid_fire):
        Update=False
        cutime=max(Grid_fire.values()) 
        for grid,time in Grid_fire.items():
            x,y=grid
            self.TG.declare(DetectFire(id=(x,y),yes=True,time=time))
            if cutime <self.EFA[x,y]-self.theta:  
                Update=True
##########################################################################################     
        if not Update:
            self.TG.run()
        return tasks, Update, self.latime, cutime
    def EventUpdateFT(self, nEFA,time):
        # nEFA[x,y]>0 and  nEFA[x,y]<255])
        nEFA[nEFA>self.plantime]=0
        x,y=np.where(nEFA>0)
        for i in range(len(x)):
            if nEFA[x[i],y[i]]+time <self.EFA[x[i],y[i]]:
                self.TG.declare(UpdateEFA(id=(x[i],y[i]), NEFA=nEFA[x[i],y[i]], ct=time))  
        self.TG.run()
        return tasks
class TaskGenerator(KnowledgeEngine):
    @DefFacts()
    def startup(self):
        yield(Theta(v=9, plan=20))
        yield (Time(Timer))
        yield(Phase2(-1))
    ########## For maintain a timer 
    @Rule(DetectFire(time=MATCH.t,id=MATCH.id), AS.f<< Time(MATCH.ct),salience=1)
    def CheckTime(self,t,f,ct):
        if ct<t:
            self.modify(f,_0=t)
    ########## For reduce redundancy
    @Rule(AS.f<<DetectFire(id=MATCH.id,yes=True) & Grid(id=MATCH.id, state='B'),salience=1)
    def Remove_Dup(self,f,id):
        self.retract(f)
    @Rule(AS.f<<DetectFire(id=MATCH.id,yes=False) & (Grid(id=MATCH.id, state='N')|Grid(id=MATCH.id, state='O')),salience=1)
    def Remove_Dup(self,f,id):
        self.retract(f)
    ##############ChangeState
    @Rule (AS.f1<<DetectFire(id=MATCH.id, yes=False,time=MATCH.t) & AS.f<<(Grid(state='UK', id=MATCH.id)))
    def ShiftUKtoN(self,f,f1,id,t):
        self.retract(f1)
        self.modify(f,state='N',time=t)
        tasks[id].pop('FD', None)
    @Rule (AS.f1<<DetectFire(id=MATCH.id, yes=True,time=MATCH.t) & AS.f<<(Grid(state='UK', id=MATCH.id)))
    def ShiftUKtoB(self,f,f1,id,t):
        self.retract(f1)
        self.modify(f,state='B',time=t)
        tasks[id].pop('FD', None)
    @Rule (AS.f1<<DetectFire(id=MATCH.id, yes=False,time=MATCH.t) & AS.f<<(Grid(state='B', id=MATCH.id)))
    def ShiftBtoO(self,f,f1,id,t):
        self.retract(f1)
        self.modify(f,state='O',time=t)
        tasks[id].pop('IM', None)
    @Rule (AS.f1<<DetectFire(id=MATCH.id, yes=True,time=MATCH.t) & AS.f<<(Grid(state='N', id=MATCH.id)))
    def ShiftNtoB(self,f,f1,id,t):
        self.retract(f1)
        self.modify(f,state='B',time=t)
        tasks[id].pop('FD', None)
        tasks[id].pop('FT', None)
        tasks[id].pop('BM',None)
    ############ Add task Fire detection
    @Rule (AS.f<<Grid(state='UK',id=MATCH.i,time=MATCH.t))
    def FD(self,f,i,t):
        tasks[i]['FD']=(t,-1)
    #############Enter Phase 2:
    @Rule(AS.f<<Phase2(L(-1))&FORALL(Grid(id=MATCH.i),Grid(id=MATCH.i,state=~L('UK')))&Time(MATCH.ct))
    def all_Grid_passed(self,ct,f):
        print(f"=====Get into Phase 2=====  at {ct}")
        self.modify(f,_0=ct)
    @Rule (Phase2(~L(-1))&Theta(v=MATCH.v, plan=MATCH.plan) & AS.f<<Grid(state='N',id=MATCH.id, time=MATCH.t, EFA=MATCH.EFA) & Phase2(MATCH.ct))
    def FT(self,f,id,t,EFA,ct,v, plan):
        if max(t,ct)>=EFA-v:
            tasks[id]['FT']=(max(t,ct),-1)
        elif EFA==255 or EFA-v>=ct+plan:
            tasks[id]['BM']=(max(t,ct), -1)
        else: 
            tasks[id]['BM']=(max(t,ct), EFA-v)
            tasks[id]['FT']=(EFA-v, -1)
    ######### Add task for Intensity Monitor
    @Rule (Phase2(~L(-1))& AS.f<<Grid(state='B',id=MATCH.id,time=MATCH.t)& Phase2(MATCH.ct))
    def FI(self,f,id,t,ct):
        tasks[id]['FI']=(max(t,ct),-1)
    ##############################################
    @Rule (Phase2(~L(-1))& Theta(v=MATCH.v, plan=MATCH.plan) & AS.f<<Grid(state='N',id=MATCH.id, EFA=MATCH.EFA, time=MATCH.time)& AS.f1<< UpdateEFA(id=MATCH.id, NEFA=MATCH.NEFA, ct=MATCH.ct))
    def UpdateFT(self,f, f1,v, id, EFA, time,NEFA, ct):
        NEFA=NEFA+ct
        self.retract(f1)
        self.modify(f,EFA=NEFA, time=ct)
        if NEFA<EFA:
            tasks[id]['FT']=(max(ct,NEFA-v),-1)
        try:
            if ct>=NEFA-v:
                tasks[id].pop('BM')
            else:
                x,y=tasks[id]['BM']
                tasks[id]['BM']=(x,max(ct,NEFA-v))
        except:
            pass
    ############################################################
    # @Rule (Phase2(~L(-1))& Mission(step=MATCH.step, BM=MATCH.BM, FT=MATCH.FT)&Theta(v=MATCH.v, plan=MATCH.plan) & AS.f<<Grid(state='N',id=MATCH.id, time=MATCH.t, EFA=MATCH.EFA) & Phase2(MATCH.ct))
    # def FT(self,f,id,t,EFA,ct,v, plan,step, BM, FT ):
    #     if max(t,ct)>=EFA-v:
    #         #print(f" see {step} {BM} {FT}")
    #         tasks[id]['FT']=(max(t,ct),-1)
    #     elif EFA==255 or EFA-v>=ct+plan:
    #         tasks[id]['BM']=(max(t,ct), -1)
    #     else: 
    #         if EFA-v-(max(t,ct))>BM:
    #             dd=(EFA-v-(max(t,ct)))//BM*BM+max(t,ct)
    #             tasks[id]['BM']=(max(t,ct), dd)
    #         #tmp=min(step,FT)
    #         st=EFA//step*step-v
    #         tasks[id]['FT']=(st, -1)
    #         #print(f"EFA {EFA} {EFA-v} {plan} ")
    #         print(f"add task {tasks[id]}")
    #     #print(f"Generate task FT at grid {id}")
Timer=0
tasks=defaultdict(dict)
#Try=2

'''
#print(TG.facts)
print(f"TASKS: {tasks}")
TG.declare(DetectFire(id=0,yes=True,time=4))
print("===3 run===")
TG.run()
#print(TG.facts)
print(f"TASKS: {tasks}")
TG.declare(DetectFire(id=1,yes=False,time=5))
TG.declare(DetectFire(id=2,yes=True,time=5))
print("===4 run===")
TG.run()
#print(TG.facts)
print(f"TASKS: {tasks}")
TG.declare(DetectFire(id=2,yes=True,time=6))
TG.declare(DetectFire(id=2,yes=False,time=8))
print("===5 run===")
TG.run()
#print(TG.facts)
print(f"TASKS: {tasks}")
print(TG.facts)

'''



