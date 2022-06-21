# Drone_SRDS

## 1. Create Building Model (We use the DBH building as an example)
<img src="building_sample.png" width="500"> 

### 1) Building structure file (see file: data/dbh_stu.csv). 

```
x y width hight floor f_h win_below win_h fac angle clock nom
0 58.58 32.86 32.86 12 3 1.1 1.8 1,2,3 0,90,180 1,-1,-1 1/0/0,0/1/0,-1/0/0
3.53 25.72 64.85 25.72 12 3 1.1 1.8 4,5,6,7 180,270,0,90 -1,1,1,-1 -1/0/0,0/-1/0,1/0/0,0/1/0
```
DHB contains 2 rectangles. Here we describe each rectangle by specifying: 1) x, y is the coordinate of the left lower endpoint, 2) 'width' and 'height' of the rectangle, 3) floor indicates the number of floors, 4) win_below is the gap between the window bottom and the floor, 5) win_h is the height of the window, 6) fac: DBH has 7 facades, 'fac'=1,2,3 means this rectangle contains facades 1,2 and 3, 7) angle indicates the direction of each facade, 8)clock =1 iff window's id increase as the raise of y or x-axis value, otherwise clock=-1, and 9) nom give the direction vector of each facade, where 1/0/0 denotes vector (1,0,0).

### 2) Window information in one layer (see file: data/layer_win.csv). 

```
id v_1_x v_1_y v_2_x v_2_y face fix_aix angle
0 32.81 36.53 32.81 32.62 1 x 0
1 32.81 43.4 32.81 39.49 1 x 0
...
4 15.83 58.37 20.22 58.37 2 y 90
5 9.35 58.37 13.54 58.37 2 y 90
...
31 43.97 25.56 47.97 25.56 7 y 90
```
This file gives the window ID, and uses 'v_1_x v_1_y v_2_x v_2_y' to specific the 2D coordinate of each window. 'face' tells the window in which facade, and 'fix_aix' gives its coordinates don't change in which axis, and 'angle' shows the direction of the facade. 

### 3) Room information in one layer (see file: data/rooms.csv). 

```
r,w,d,h
0,6.5,14,3
1,3,4,3
...
42,3,4,3
```
This file give the Room ID, and its width, depth and height. 

### 4) Correlation between room and window (see file: data/window_room.csv). 
```
w r
0 0
...
3 3,s1
4 4,5
...
30 41
```
This file tells which room each window locates in (two rooms may share one window). 




