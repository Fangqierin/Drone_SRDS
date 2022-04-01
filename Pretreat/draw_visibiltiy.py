import pyvisgraph as vg
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
building=pd.read_csv('dbh_stu.csv',delimiter=' ')
polygens=[]
polygen=[]
w=[];h=[];node=[]
R_v=3
off=3
for index,row in building.iterrows():
    polygen=[]
    w.append(row['width'])
    h.append(row['hight'])
    node.append(row.loc[['x','y']].values)
polygen=[]
# polygen.append(vg.Point(node[0][0],node[0][1]))
# polygen.append(vg.Point(node[0][0],node[0][1]+h[0]+h[1]))
# polygen.append(vg.Point(node[0][0]+w[1],node[0][1]+h[0]+h[1]))
# polygen.append(vg.Point(node[0][0]+w[1],node[0][1]+h[0]))
# polygen.append(vg.Point(node[0][0]+w[0],node[0][1]+h[0]))
# polygen.append(vg.Point(node[0][0]+w[0],node[0][1]))
polygen.append(vg.Point(node[0][0],node[0][1]))
polygen.append(vg.Point(node[0][0],node[0][1]-h[0]))
polygen.append(vg.Point(node[1][0],node[1][1]))
polygen.append(vg.Point(node[1][0],node[1][1]-h[1]))
polygen.append(vg.Point(node[1][0]+w[1],node[1][1]-h[1]))
polygen.append(vg.Point(node[1][0]+w[1],node[1][1]))
polygen.append(vg.Point(node[0][0]+w[0],node[0][1]-h[0]))
polygen.append(vg.Point(node[0][0]+w[0],node[0][1]))
plt.plot([polygen[i].x for i in list(range(len(polygen)))+[0]],[polygen[i].y for i in list(range(len(polygen)))+[0]],'black')

# polygen.append(vg.Point(node[0][0]+w[1],node[0][1]+h[0]+h[1]))
# polygen.append(vg.Point(node[0][0]+w[1],node[0][1]+h[0]))
# polygen.append(vg.Point(node[0][0]+w[0],node[0][1]+h[0]))
# polygen.append(vg.Point(node[0][0]+w[0],node[0][1]))

#polys = [[vg.Point(0.0,1.0), vg.Point(3.0,1.0), vg.Point(1.5,4.0)],[vg.Point(4.0,4.0), vg.Point(7.0,4.0), vg.Point(5.5,8.0)]]

polygen=[]
polygen.append(vg.Point(node[0][0]-off,node[0][1]+off))
polygen.append(vg.Point(node[0][0]-off,node[0][1]-h[0]-off))
polygen.append(vg.Point(node[1][0]-off,node[1][1]-off))
polygen.append(vg.Point(node[1][0]-off,node[1][1]-h[1]-off))
polygen.append(vg.Point(node[1][0]+w[1]+off,node[1][1]-h[1]-off))
polygen.append(vg.Point(node[1][0]+w[1]+off,node[1][1]+off))
polygen.append(vg.Point(node[0][0]+w[0]+off,node[0][1]-h[0]+off))
polygen.append(vg.Point(node[0][0]+w[0]+off,node[0][1]+off))
graph=vg.VisGraph()
graph.build([polygen])
#shortest = graph.shortest_path(vg.Point(-5,54.84), vg.Point(84.38,5.54))
plt.plot([polygen[i].x for i in list(range(len(polygen)))+[0]],[polygen[i].y for i in list(range(len(polygen)))+[0]],'g--')
shortest = graph.shortest_path( vg.Point(0,-10),vg.Point(50,35))
#print(f"see 1 {[shortest[i].x for i in range(len(shortest))],[shortest[i].y for i in range(len(shortest))]},")
z1=2; z2=32
length=0
split=[0]
for i in range(len(shortest)-1):
    tmp=np.sum((np.array([shortest[i].x,shortest[i].y])-np.array([shortest[i+1].x,shortest[i+1].y]))**2, axis=0)
    length=length+round(np.sqrt(tmp)/R_v,2)
    split.append(length)
height=[i/length*(z2-z1)+z1 for i in split]
graph.save('polygen.graph')
graph.load('polygen.graph')
plt.plot([shortest[i].x for i in range(len(shortest))],[shortest[i].y for i in range(len(shortest))],'b',linewidth=2)
plt.plot([shortest[i].x for i in range(len(shortest))],[shortest[i].y for i in range(len(shortest))],'b.',markersize=12)
print(f"see 1 {[shortest[i].x for i in range(len(shortest))],[shortest[i].y for i in range(len(shortest))]} {height}")
shortest = graph.shortest_path(vg.Point(0,-10),vg.Point(20,68))
z1=2; z2=20
length=0
split=[0]
for i in range(len(shortest)-1):
    tmp=np.sum((np.array([shortest[i].x,shortest[i].y])-np.array([shortest[i+1].x,shortest[i+1].y]))**2, axis=0)
    length=length+round(np.sqrt(tmp)/R_v,2)
    split.append(length)
height=[i/length*(z2-z1)+z1 for i in split]
plt.plot([shortest[i].x for i in range(len(shortest))],[shortest[i].y for i in range(len(shortest))],'orange',linewidth=2)
plt.plot([shortest[i].x for i in range(len(shortest))],[shortest[i].y for i in range(len(shortest))],'y.',markersize=12)
print(f"see 2 {[shortest[i].x for i in range(len(shortest))],[shortest[i].y for i in range(len(shortest))]} {height}")
shortest = graph.shortest_path(vg.Point(0,-10),vg.Point(40,-15))
z1=2; z2=30
length=0
split=[0]
for i in range(len(shortest)-1):
    tmp=np.sum((np.array([shortest[i].x,shortest[i].y])-np.array([shortest[i+1].x,shortest[i+1].y]))**2, axis=0)
    length=length+round(np.sqrt(tmp)/R_v,2)
    split.append(length)
height=[i/length*(z2-z1)+z1 for i in split]
plt.plot([shortest[i].x for i in range(len(shortest))],[shortest[i].y for i in range(len(shortest))],'green',linewidth=2)
plt.plot([shortest[i].x for i in range(len(shortest))],[shortest[i].y for i in range(len(shortest))],'g.',markersize=12)
print(f"see 3 {[shortest[i].x for i in range(len(shortest))],[shortest[i].y for i in range(len(shortest))]} {height}")
plt.xlim(-4,72)
plt.ylim(-16,70)
plt.show()
