'''
Created on Jul 31, 2020

@author: fangqiliu
'''
import numpy as np
import random as rm

states = ["Sleep","Icecream","Run"]

# Possible sequences of events
transitionName = [["SS","SR","SI"],["RS","RR","RI"],["IS","IR","II"]]

# Probabilities matrix (transition matrix)
transitionMatrix = [[0.2,0.6,0.2],[0.1,0.6,0.3],[0.2,0.7,0.1]]
if sum(transitionMatrix[0])+sum(transitionMatrix[1])+sum(transitionMatrix[1]) != 3:
    print("Somewhere, something went wrong. Transition matrix, perhaps?")
else: print("All is gonna be okay, you should move on!! ;)")
def activity_forecast(days):
    # Choose the starting state
    activityToday = "Sleep"
    print("Start state: " + activityToday)
    # Shall store the sequence of states taken. So, this only has the starting state for now.
    activityList = [activityToday]
    i = 0
    # To calculate the probability of the activityList
    prob = 1
    while i != days:
        if activityToday == "Sleep":
            change = np.random.choice(transitionName[0],replace=True,p=transitionMatrix[0])
            if change == "SS":
                prob = prob * 0.2
                activityList.append("Sleep")
                pass
            elif change == "SR":
                prob = prob * 0.6
                activityToday = "Run"
                activityList.append("Run")
            else:
                prob = prob * 0.2
                activityToday = "Icecream"
                activityList.append("Icecream")
        elif activityToday == "Run":
            change = np.random.choice(transitionName[1],replace=True,p=transitionMatrix[1])
            if change == "RR":
                prob = prob * 0.5
                activityList.append("Run")
                pass
            elif change == "RS":
                prob = prob * 0.2
                activityToday = "Sleep"
                activityList.append("Sleep")
            else:
                prob = prob * 0.3
                activityToday = "Icecream"
                activityList.append("Icecream")
        elif activityToday == "Icecream":
            change = np.random.choice(transitionName[2],replace=True,p=transitionMatrix[2])
            if change == "II":
                prob = prob * 0.1
                activityList.append("Icecream")
                pass
            elif change == "IS":
                prob = prob * 0.2
                activityToday = "Sleep"
                activityList.append("Sleep")
            else:
                prob = prob * 0.7
                activityToday = "Run"
                activityList.append("Run")
        i += 1  
    print("Possible states: " + str(activityList))
    print("End state after "+ str(days) + " days: " + activityToday)
    print("Probability of the possible sequence of states: " + str(prob))

# Function that forecasts the possible state for the next 2 days
activity_forecast(2)
#########Input 
#Office room:
# Dimension: 4 m􏰍5 m􏰍3 m (W􏰍D􏰍H)
D=[4,5,3]
The fuel density: mean 1⁄4 24.8 kg/m2; standard tion 1⁄4 8.6 kg/m2 [23].
fu_d=24.8 
Heat of combustion of fuel Hch 1⁄4 12.4 KJ/g [27] Fire growth parameter: a1⁄40.0029 kW/s2 (slow)
devia-
􏰋 Window:
Dimension: 1.0 m 􏰍 1.0 m (W 􏰍 H),
Material: common glass;
Fire resistance rate to ISO 834 fire: mean1⁄42 min, standard deviation1⁄40.3 min (assumed).
􏰋 Door:
Dimension: 1.0 m 􏰍 2.0 m (W 􏰍 H),
Material: wood;
Fire resistance rate to ISO 834 fire: mean1⁄410 min, stan- dard deviation1⁄41.5 min (assumed).
􏰋 Wall and ceiling: pffiffiffiffiffiffiffiffi 2 0:5 Material: gypsum board krc 1⁄4 0:742 KJ=m s K [23]
Fire resistance rating to ISO 834 fire: standard deviation1⁄49 min (assumed).
􏰋 Floor:
Material: normal weight concrete
[25]
Fire resistance rating to ISO 834 fire: mean1⁄490 min, standard deviation 1⁄4 13.5 min (assumed).
􏰋 Stairwell:
Dimension: 5􏰍3􏰍3 m (W􏰍D􏰍H)
The fuel density: mean 1⁄4 0 kg/m2, standard deviation 1⁄4 0 kg/m2.

















































